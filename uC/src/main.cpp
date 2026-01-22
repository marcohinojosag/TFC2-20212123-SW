#include <Arduino.h>
#include <SPI.h>

// ================== TMAG5170 (SPI) ==================
static const int TMAG_SCK  = 18;
static const int TMAG_MISO = 19;
static const int TMAG_MOSI = 23;
static const int TMAG_CS   = 22;

static const uint8_t REG_DEVICE_CONFIG    = 0x00;
static const uint8_t REG_SENSOR_CONFIG    = 0x01;
static const uint8_t REG_SYSTEM_CONFIG    = 0x02;
static const uint8_t REG_ANGLE_RESULT     = 0x13;
static const uint8_t REG_MAGNITUDE_RESULT = 0x14;

static const uint32_t TMAG_SPI_HZ = 1000000;
static const uint8_t  TMAG_SPI_MODE = SPI_MODE0;

// CRC4 (poly x^4 + x + 1, init 0xF), sobre bits [31:4]
static uint8_t crc4_28(uint32_t frame_no_crc) {
  uint8_t crc = 0xF;
  for (int i = 31; i >= 4; --i) {
    uint8_t bit = (frame_no_crc >> i) & 0x1;
    uint8_t fb = ((crc >> 3) & 0x1) ^ bit;
    crc = ((crc << 1) & 0xF);
    if (fb) crc ^= 0x3;
  }
  return crc & 0xF;
}

static uint32_t tmagBuildFrame(bool isRead, uint8_t addr, uint16_t data, uint8_t cmd = 0x0) {
  uint32_t f = 0;
  f |= (isRead ? 1UL : 0UL) << 31;
  f |= (uint32_t)(addr & 0x7F) << 24;
  f |= (uint32_t)data << 8;
  f |= (uint32_t)(cmd & 0x0F) << 4;
  uint32_t f0 = (f & 0xFFFFFFF0);
  f0 |= (uint32_t)crc4_28(f0);
  return f0;
}

static uint32_t tmagXfer32(uint32_t tx) {
  SPI.beginTransaction(SPISettings(TMAG_SPI_HZ, MSBFIRST, TMAG_SPI_MODE));
  digitalWrite(TMAG_CS, LOW);

  uint32_t rx = 0;
  rx |= (uint32_t)SPI.transfer((tx >> 24) & 0xFF) << 24;
  rx |= (uint32_t)SPI.transfer((tx >> 16) & 0xFF) << 16;
  rx |= (uint32_t)SPI.transfer((tx >>  8) & 0xFF) <<  8;
  rx |= (uint32_t)SPI.transfer((tx >>  0) & 0xFF) <<  0;

  digitalWrite(TMAG_CS, HIGH);
  SPI.endTransaction();
  return rx;
}

static void tmagWriteReg(uint8_t addr, uint16_t val) {
  (void)tmagXfer32(tmagBuildFrame(false, addr, val, 0x0));
  delayMicroseconds(5);
}

static uint16_t tmagReadReg(uint8_t addr) {
  uint32_t rx = tmagXfer32(tmagBuildFrame(true, addr, 0x0000, 0x0));
  return (uint16_t)((rx >> 8) & 0xFFFF);
}

static void tmagConfigAngleXY() {
  // Active measure + promedio 2x
  tmagWriteReg(REG_DEVICE_CONFIG, 0x1020);
  // ANGLE_EN=XY + MAG_CH_EN=X,Y + rangos X/Y ±25mT (ajustable)
  tmagWriteReg(REG_SENSOR_CONFIG, 0x40C5);
  tmagWriteReg(REG_SYSTEM_CONFIG, 0x0000);
  delay(5);
}

static float tmagReadAngleDeg() {
  uint16_t a = tmagReadReg(REG_ANGLE_RESULT);
  return (float)(a & 0x1FFF) / 16.0f;
}

// ================== VNH7070 (GPIO) ==================
static const int VNH_INA  = 21;
static const int VNH_INB  = 4;
static const int VNH_SEL0 = 16;
static const int VNH_PWM  = 17;

static const int VNH_CS_ADC = 34;   // ADC1 (bien para ESP32)

// PWM LEDC
static const int PWM_CH   = 0;
static const int PWM_FREQ = 20000;
static const int PWM_RES  = 10;     // 0..1023

static float g_duty = 0.0f;

// Para corriente
static const float R_CS_OHMS = 1000.0f;   // tu R9
static float K_TYP = 1540.0f;            // aproximación inicial (calibrable)

// Promedio simple para estabilizar lecturas
static uint32_t readmV_avg(int pin, int n = 16) {
  uint32_t acc = 0;
  for (int i = 0; i < n; ++i) {
    acc += analogReadMilliVolts(pin);
    delayMicroseconds(200);
  }
  return acc / (uint32_t)n;
}

static void vnhInit() {
  pinMode(VNH_INA, OUTPUT);
  pinMode(VNH_INB, OUTPUT);
  pinMode(VNH_SEL0, OUTPUT);

  // SEL0 HIGH: modo normal (y CS activo como sense/diag según el chip)
  digitalWrite(VNH_SEL0, HIGH);

  // PWM
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(VNH_PWM, PWM_CH);

  // ADC (GPIO34)
  analogSetPinAttenuation(VNH_CS_ADC, ADC_11db); // hasta ~3.3V

  // Estado inicial: motor parado
  ledcWrite(PWM_CH, 0);
  digitalWrite(VNH_INA, LOW);
  digitalWrite(VNH_INB, LOW);
}

static void vnhSetForwardDuty(float duty01) {
  duty01 = constrain(duty01, 0.0f, 1.0f);
  digitalWrite(VNH_SEL0, HIGH);
  digitalWrite(VNH_INA, HIGH);
  digitalWrite(VNH_INB, LOW);
  int maxDuty = (1 << PWM_RES) - 1;
  ledcWrite(PWM_CH, (int)lroundf(duty01 * maxDuty));
}

static void vnhStop() {
  ledcWrite(PWM_CH, 0);
  digitalWrite(VNH_INA, LOW);
  digitalWrite(VNH_INB, LOW);
}

static float vnhReadCurrentA(int avgN = 16) {
  float vcs = (float)readmV_avg(VNH_CS_ADC, avgN) / 1000.0f; // V en R9
  float iSense = vcs / R_CS_OHMS;                            // A (corriente CS)
  float iOut = K_TYP * iSense;                               // A (aprox)
  return iOut;
}

// ================== Serial commands ==================
static void handleCmd(const String &cmd) {
  if (cmd.length() == 0) return;

  if (cmd == "S") {
    g_duty = 0.0f;
    vnhStop();
    Serial.println("OK: STOP");
    return;
  }

  if (cmd.startsWith("M")) {
    int pct = cmd.substring(1).toInt();
    pct = constrain(pct, 0, 100);
    g_duty = pct / 100.0f;
    vnhSetForwardDuty(g_duty);
    Serial.printf("OK: duty=%.2f\n", g_duty);
    return;
  }

  if (cmd == "R") {
    float ang = tmagReadAngleDeg();
    uint16_t mag = tmagReadReg(REG_MAGNITUDE_RESULT);
    float iA = vnhReadCurrentA(16);
    uint32_t vcs_mV = readmV_avg(VNH_CS_ADC, 16);

    Serial.printf("angle=%.2f deg | mag=0x%04X | Vcs=%lumV | I=%.2fA | duty=%.2f\n",
                  ang, mag, (unsigned long)vcs_mV, iA, g_duty);
    return;
  }

  Serial.println("Comandos: Mxx (0-100), S, R");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // SPI init + TMAG
  pinMode(TMAG_CS, OUTPUT);
  digitalWrite(TMAG_CS, HIGH);
  SPI.begin(TMAG_SCK, TMAG_MISO, TMAG_MOSI, -1);
  tmagConfigAngleXY();

  // Driver init
  vnhInit();
  vnhSetForwardDuty(0.0f);

  Serial.println("Listo. Comandos: Mxx (0-100), S, R");
}

void loop() {
  // Mantener motor en el último duty
  vnhSetForwardDuty(g_duty);

  // Leer comandos por Serial
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      buf.trim();
      handleCmd(buf);
      buf = "";
    } else {
      buf += c;
    }
  }

  delay(10);
}
