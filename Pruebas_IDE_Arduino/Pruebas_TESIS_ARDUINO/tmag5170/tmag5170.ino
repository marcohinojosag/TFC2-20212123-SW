#include <Arduino.h>
#include <SPI.h>

// =========================================================
// ===================== VNH7070BAS ========================
// =========================================================
const int INA  = 21;
const int INB  = 4;
const int SEL0 = 16;
const int PWM_PIN  = 17;
const int CS_ADC   = 34;   // ADC1 (corriente sense)

// PWM
const int PWM_FREQ = 20000;
const int PWM_RES  = 10;
const int PWM_CH   = 0;    // solo core viejo

// Corriente (CS)
const float RSENSE_OHMS = 1000.0f;  // R9=1k
const float K_TYP = 1540.0f;        // aproximación (calibrable)

// Boost (opcional)
const bool USE_START_BOOST = true;
const int  BOOST_THRESHOLD_PCT = 60;
const int  BOOST_PCT = 80;
const uint32_t BOOST_MS = 120;

static inline int maxDuty() { return (1 << PWM_RES) - 1; }

// Compat LEDC (core ESP32 v2 vs v3)
static void pwmInit() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
#else
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CH);
#endif
}

static void pwmWrite01(float duty01) {
  duty01 = constrain(duty01, 0.0f, 1.0f);
  int ticks = (int)lroundf(duty01 * maxDuty());
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(PWM_PIN, ticks);
#else
  ledcWrite(PWM_CH, ticks);
#endif
}

void vnhForward() {
  digitalWrite(SEL0, HIGH);
  digitalWrite(INA, HIGH);
  digitalWrite(INB, LOW);
}

void vnhStop() {
  pwmWrite01(0.0f);
  digitalWrite(INA, LOW);
  digitalWrite(INB, LOW);
}

float readCurrentA_1shot() {
  uint16_t mv = (uint16_t)analogReadMilliVolts(CS_ADC);
  float vcs = mv / 1000.0f;
  float iSense = vcs / RSENSE_OHMS;
  return K_TYP * iSense;
}

// =========================================================
// ===================== TMAG5170A1 =========================
// =========================================================
static const int TMAG_SCK  = 18;
static const int TMAG_MISO = 19;
static const int TMAG_MOSI = 23;
static const int TMAG_CS   = 22;

static const uint32_t TMAG_SPI_HZ = 1000000;
static const uint8_t  TMAG_SPI_MODE = SPI_MODE0;

static const uint8_t REG_DEVICE_CONFIG = 0x00;
static const uint8_t REG_SENSOR_CONFIG = 0x01;
static const uint8_t REG_SYSTEM_CONFIG = 0x02;
static const uint8_t REG_ANGLE_RESULT  = 0x13;

// CRC4 TI (init=0xF)
static uint8_t crc4_ti_from_frame_no_crc(uint32_t frame_no_crc) {
  uint32_t frame28 = (frame_no_crc >> 4) & 0x0FFFFFFF;
  uint8_t crc = 0xF;
  uint32_t padded = (frame28 << 4);
  for (int i = 31; i >= 0; --i) {
    uint8_t inv = ((padded >> i) & 1U) ^ ((crc >> 3) & 1U);
    uint8_t c3 = (crc >> 2) & 1U;
    uint8_t c2 = (crc >> 1) & 1U;
    uint8_t c1 = (crc >> 0) & 1U;
    crc = (c3 << 3) | (c2 << 2) | ((c1 ^ inv) << 1) | (inv);
  }
  return crc & 0x0F;
}

// SDI: [31]=R/W [30:24]=ADDR [23:8]=DATA [7:4]=CMD [3:0]=CRC
static uint32_t buildSDI(bool isRead, uint8_t addr, uint16_t data, uint8_t cmdNibble = 0x0) {
  uint32_t f = 0;
  f |= (isRead ? 1UL : 0UL) << 31;
  f |= (uint32_t)(addr & 0x7F) << 24;
  f |= (uint32_t)data << 8;
  f |= (uint32_t)(cmdNibble & 0x0F) << 4;
  f &= 0xFFFFFFF0;
  f |= (uint32_t)crc4_ti_from_frame_no_crc(f);
  return f;
}

static uint32_t tmagTransfer32(uint32_t tx) {
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

static uint16_t sdoData16(uint32_t rx) { return (uint16_t)((rx >> 8) & 0xFFFF); }

static bool sdoCrcOk(uint32_t rx) {
  uint32_t no_crc = rx & 0xFFFFFFF0;
  uint8_t calc = crc4_ti_from_frame_no_crc(no_crc);
  uint8_t got  = (uint8_t)(rx & 0x0F);
  return (calc == got);
}

static void tmagWriteReg(uint8_t addr, uint16_t val) {
  (void)tmagTransfer32(buildSDI(false, addr, val, 0x0));
  delayMicroseconds(5);
}

static uint16_t tmagReadReg(uint8_t addr, uint32_t *rawRxOut = nullptr) {
  uint32_t rx = tmagTransfer32(buildSDI(true, addr, 0x0000, 0x0));
  if (rawRxOut) *rawRxOut = rx;
  return sdoData16(rx);
}

static void tmagConfigAngleXY() {
  // Active measure + avg 2x
  tmagWriteReg(REG_DEVICE_CONFIG, 0x1020);
  // ANGLE_EN=XY, MAG_CH_EN=X,Y, ranges default ±50mT
  tmagWriteReg(REG_SENSOR_CONFIG, 0x40C0);
  tmagWriteReg(REG_SYSTEM_CONFIG, 0x0000);
  delay(5);
}

// =========================================================
// ===================== Captura conjunta ==================
// =========================================================
static const uint32_t CAPTURE_MS = 5000;
static const uint16_t SAMPLE_DELAY_MS = 10; // ~100 Hz

static float prevAngle = 0.0f;
static uint32_t prevUs = 0;

static float unwrapDeltaDeg(float a, float aPrev) {
  float d = a - aPrev;
  if (d > 180.0f) d -= 360.0f;
  if (d < -180.0f) d += 360.0f;
  return d;
}

static void printHeader(int dutyPct, bool boost) {
  Serial.println();
  Serial.println("INICIO CAPTURA (MOTOR + ANGULO) - 5s");
  Serial.printf("Duty solicitado: %d%% | Boost: %s\n", dutyPct, boost ? "YES" : "NO");
  Serial.println("Columnas (numericas, separadas por espacios):");
  Serial.println("1) t_us        (microsegundos desde inicio)");
  Serial.println("2) angle_deg   (grados 0-360)");
  Serial.println("3) omega_rad_s (rad/s)");
  Serial.println("4) I_est_A     (A, estimada desde CS_ADC)");
  Serial.println("5) crcOK       (1 ok, 0 mal)");
  Serial.println();
  Serial.println("t_us angle_deg omega_rad_s I_est_A crcOK");
}

static void runMotorAndCapture5s(int dutyPct) {
  dutyPct = constrain(dutyPct, 0, 100);
  float dutyReq = dutyPct / 100.0f;

  bool doBoost = USE_START_BOOST && (dutyPct > 0) && (dutyPct < BOOST_THRESHOLD_PCT);
  uint32_t boostEndUs = doBoost ? (BOOST_MS * 1000UL) : 0;

  // Start motor
  vnhForward();

  // Reset omega state
  prevUs = 0;
  prevAngle = 0;

  uint32_t startMs = millis();
  uint32_t startUs = micros();

  printHeader(dutyPct, doBoost);

  while ((millis() - startMs) < CAPTURE_MS) {
    // (Opcional) abortar con 'S'
    if (Serial.available()) {
      char c = (char)Serial.read();
      if (c == 'S' || c == 's') {
        Serial.println("# CAPTURA ABORTADA");
        vnhStop();
        return;
      }
    }

    uint32_t nowUs = micros();
    uint32_t dtUs = nowUs - startUs;

    // PWM control con boost al inicio
    if (doBoost && dtUs < boostEndUs) pwmWrite01(BOOST_PCT / 100.0f);
    else pwmWrite01(dutyReq);

    // TMAG read
    uint32_t rxA=0;
    uint16_t a = tmagReadReg(REG_ANGLE_RESULT, &rxA);
    float angleDeg = (float)(a & 0x1FFF) / 16.0f;

    // omega
    float omega = 0.0f;
    if (prevUs != 0) {
      float dt = (nowUs - prevUs) * 1e-6f;
      float ddeg = unwrapDeltaDeg(angleDeg, prevAngle);
      omega = (ddeg * (3.1415926f / 180.0f)) / (dt > 1e-6f ? dt : 1e-6f);
    }
    prevUs = nowUs;
    prevAngle = angleDeg;

    // Corriente
    float iA = readCurrentA_1shot();

    // CRC OK
    int ok = sdoCrcOk(rxA) ? 1 : 0;

    // Print (espacios)
    Serial.print(dtUs); Serial.print(" ");
    Serial.print(angleDeg, 2); Serial.print(" ");
    Serial.print(omega, 3); Serial.print(" ");
    Serial.print(iA, 4); Serial.print(" ");
    Serial.println(ok);

    delay(SAMPLE_DELAY_MS);
  }

  // Stop motor at end
  vnhStop();
  Serial.println("# FIN CAPTURA (motor detenido)");
  Serial.println();
}

// =========================================================
// ===================== Serial command ====================
// =========================================================
static void handleCmd(String cmd) {
  cmd.trim();
  if (!cmd.length()) return;

  if (cmd == "S") {
    vnhStop();
    Serial.println("OK STOP");
    return;
  }

  if (cmd.startsWith("M")) {
    int pct = cmd.substring(1).toInt();
    pct = constrain(pct, 0, 100);
    Serial.printf("OK M%d -> captura 5s + stop\n", pct);
    runMotorAndCapture5s(pct);
    return;
  }

  Serial.println("Comandos: Mxx (0-100)  |  S");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // VNH pins
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(SEL0, OUTPUT);
  digitalWrite(SEL0, HIGH);

  pwmInit();
  analogSetPinAttenuation(CS_ADC, ADC_11db);
  vnhStop();

  // TMAG SPI
  pinMode(TMAG_CS, OUTPUT);
  digitalWrite(TMAG_CS, HIGH);
  SPI.begin(TMAG_SCK, TMAG_MISO, TMAG_MOSI, -1);
  tmagConfigAngleXY();

  Serial.println("Listo: VNH7070 + TMAG5170.");
  Serial.println("Comandos: Mxx (0-100)  |  S (stop/abort)");
}

void loop() {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      handleCmd(buf);
      buf = "";
    } else {
      buf += c;
    }
  }
  delay(5);
}

