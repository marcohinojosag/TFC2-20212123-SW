#include <Arduino.h>
#include <SPI.h>

// ===== SPI ESP32 (VSPI) =====
static const int PIN_SCK  = 18;
static const int PIN_MISO = 19;
static const int PIN_MOSI = 23;
static const int PIN_CS   = 22;

// ===== TMAG5170 register map =====
static const uint8_t REG_DEVICE_CONFIG = 0x00;
static const uint8_t REG_SENSOR_CONFIG = 0x01;
static const uint8_t REG_SYSTEM_CONFIG = 0x02;
static const uint8_t REG_TEST_CONFIG   = 0x0F;

static const uint8_t REG_X_CH_RESULT   = 0x09;
static const uint8_t REG_Y_CH_RESULT   = 0x0A;
static const uint8_t REG_Z_CH_RESULT   = 0x0B;

// ===== SPI settings =====
static const uint32_t SPI_HZ = 1000000;
static const uint8_t  SPI_MODE_USED = SPI_MODE0;

static inline void csLow()  { digitalWrite(PIN_CS, LOW); }
static inline void csHigh() { digitalWrite(PIN_CS, HIGH); }

static uint16_t tmagRead16(uint8_t addr) {
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE_USED));
  csLow();

  SPI.transfer(addr | 0x80);        // read
  uint8_t b1 = SPI.transfer(0x00);
  uint8_t b2 = SPI.transfer(0x00);
  (void)SPI.transfer(0x00);         // dummy (CRC off)

  csHigh();
  SPI.endTransaction();
  return (uint16_t)((b1 << 8) | b2);
}

static void tmagWrite16(uint8_t addr, uint16_t data16, uint8_t lastByte = 0x00) {
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE_USED));
  csLow();

  SPI.transfer(addr & 0x7F);          // write
  SPI.transfer((data16 >> 8) & 0xFF);
  SPI.transfer((data16 >> 0) & 0xFF);
  SPI.transfer(lastByte);

  csHigh();
  SPI.endTransaction();
  delayMicroseconds(5);
}

// Deshabilitar CRC (0x0F000407)
static void disableCRC_viaTestConfig() {
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE_USED));
  csLow();
  SPI.transfer(REG_TEST_CONFIG);
  SPI.transfer(0x00);
  SPI.transfer(0x04);
  SPI.transfer(0x07);
  csHigh();
  SPI.endTransaction();
  delay(5);
}

// Config: Active measure + habilitar XYZ + rangos ±100mT (0x41EA)
static void configXYZ() {
  tmagWrite16(REG_DEVICE_CONFIG, 0x0020); // active measure
  tmagWrite16(REG_SENSOR_CONFIG, 0x41EA); // XYZ enable + ranges
  tmagWrite16(REG_SYSTEM_CONFIG, 0x0000);
  delay(10);
}

// ===== Plano para ángulo =====
enum AngleMode { ANGLE_MODE_XY, ANGLE_MODE_XZ, ANGLE_MODE_YZ };
static const AngleMode MODE = ANGLE_MODE_XY;  // cámbialo si deseas

static float angleDegFrom(int16_t x, int16_t y, int16_t z) {
  float a = 0.0f;
  switch (MODE) {
    case ANGLE_MODE_XY: a = atan2f((float)y, (float)x); break;
    case ANGLE_MODE_XZ: a = atan2f((float)z, (float)x); break;
    case ANGLE_MODE_YZ: a = atan2f((float)z, (float)y); break;
  }
  a = a * (180.0f / 3.1415926f);
  if (a < 0) a += 360.0f;
  return a;
}

// Unwrap: asegura delta en [-180, +180]
static float unwrapDeltaDeg(float aNow, float aPrev) {
  float d = aNow - aPrev;
  if (d > 180.0f) d -= 360.0f;
  if (d < -180.0f) d += 360.0f;
  return d;
}

// Estado omega
static float prevAngle = 0.0f;
static uint32_t prevUs = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_CS, OUTPUT);
  csHigh();
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);

  disableCRC_viaTestConfig();
  configXYZ();

  // Para Serial Plotter
  Serial.println("angle_deg omega_rad_s");
}

void loop() {
  int16_t x = (int16_t)tmagRead16(REG_X_CH_RESULT);
  int16_t y = (int16_t)tmagRead16(REG_Y_CH_RESULT);
  int16_t z = (int16_t)tmagRead16(REG_Z_CH_RESULT);

  float angleDeg = angleDegFrom(x, y, z);

  uint32_t nowUs = micros();
  float omegaRad = 0.0f;

  if (prevUs != 0) {
    float dt = (nowUs - prevUs) * 1e-6f;          // segundos
    float ddeg = unwrapDeltaDeg(angleDeg, prevAngle);
    float omegaDeg = ddeg / (dt > 1e-6f ? dt : 1e-6f);  // deg/s
    omegaRad = omegaDeg * (3.1415926f / 180.0f);        // rad/s
  }

  prevUs = nowUs;
  prevAngle = angleDeg;

  // Solo 2 señales para plotter
  Serial.print(angleDeg, 2);
  Serial.print(' ');
  Serial.println(omegaRad, 3);

  delay(5); // 200 Hz (si quieres fijo, luego lo cambiamos a periodo exacto)
}
