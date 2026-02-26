#include <Arduino.h>
#include <SPI.h>
#include <math.h>

// ================== VNH7070 (tus pines) ==================
static const int INA     = 21;
static const int INB     = 4;
static const int SEL0    = 16;
static const int PWM_PIN = 17;

static const int CS_ADC     = 34;   // Corriente (ADC1)
static const int VMOTOR_ADC = 35;   // Voltaje motor (ADC1) -> divisor

// ================== PWM ==================
static const int PWM_FREQ = 20000;
static const int PWM_RES  = 10;
static const int PWM_CH   = 0;

static inline uint16_t pwmMaxDuty() { return (1u << PWM_RES) - 1u; }
static inline uint16_t pctToTicks(uint8_t pct) {
  if (pct > 100) pct = 100;
  return (uint16_t)(((uint32_t)pct * (uint32_t)pwmMaxDuty() + 50u) / 100u);
}

// Compat LEDC (core ESP32 v2 vs v3)
static void pwmInit() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
#else
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CH);
#endif
}
static void pwmWriteTicks(uint16_t ticks) {
  if (ticks > pwmMaxDuty()) ticks = pwmMaxDuty();
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(PWM_PIN, ticks);   // v3: por pin
#else
  ledcWrite(PWM_CH, ticks);    // v2: por canal
#endif
}

// ================== Corriente ==================
static const uint16_t RSENSE_OHM = 1000;
static const uint16_t K_TYP      = 1540;
static inline uint32_t mvToCurrent_mA(uint16_t vcs_mV) {
  return ((uint32_t)vcs_mV * (uint32_t)K_TYP + (RSENSE_OHM / 2)) / (uint32_t)RSENSE_OHM;
}

// ================== Divisor voltaje motor (Rtop=2.9k, Rbot=970) ==================
static const uint16_t R_TOP_OHM = 2900;
static const uint16_t R_BOT_OHM = 970;
static inline uint32_t adcMvToMotorMv(uint16_t vadc_mV) {
  const uint32_t num = (uint32_t)vadc_mV * (uint32_t)(R_TOP_OHM + R_BOT_OHM);
  return (num + (R_BOT_OHM / 2)) / (uint32_t)R_BOT_OHM;
}

// ================== TMAG5170 SPI (VSPI) ==================
static const int PIN_SCK  = 18;
static const int PIN_MISO = 19;
static const int PIN_MOSI = 23;
static const int PIN_CS   = 22;

// Registros
static const uint8_t REG_DEVICE_CONFIG = 0x00;
static const uint8_t REG_SENSOR_CONFIG = 0x01;
static const uint8_t REG_SYSTEM_CONFIG = 0x02;
static const uint8_t REG_TEST_CONFIG   = 0x0F;

static const uint8_t REG_X_CH_RESULT   = 0x09;
static const uint8_t REG_Y_CH_RESULT   = 0x0A;

// SPI settings
static const uint32_t SPI_HZ = 8000000;     // 8 MHz
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
  tmagWrite16(REG_DEVICE_CONFIG, 0x0020);
  tmagWrite16(REG_SENSOR_CONFIG, 0x41EA);
  tmagWrite16(REG_SYSTEM_CONFIG, 0x0000);
  delay(10);
}

// Ángulo en XY a partir de X,Y (ya filtrados)
static inline float angleDegXYf(float x, float y) {
  float a = atan2f(y, x) * (180.0f / 3.1415926f);
  if (a < 0) a += 360.0f;
  return a;
}

// unwrap para delta (prev - now) en [-180,180]
static inline float unwrapDeltaPrevNowDeg(float aPrev, float aNow) {
  float d = aPrev - aNow;
  if (d > 180.0f) d -= 360.0f;
  if (d < -180.0f) d += 360.0f;
  return d;
}

// ================== MODOS Y TIMINGS ==================
static const uint8_t MC_STOP = 0;
static const uint8_t MC_FWD  = 1;
static const uint8_t MC_REV  = 2;

static const uint8_t CAP_CONST_FWD = 0;
static const uint8_t CAP_PROGRAM_P = 1;

// SOLO PARA MXX
static const uint32_t SAMPLE_US_M  = 250UL;
static const uint32_t CAPTURE_US_M = 2000000UL;   // 2 s

// PARA P
static const uint32_t SAMPLE_US_P  = 500UL;
static const uint32_t CAPTURE_US_P = 5000000UL;   // 5 s

// Buffers dimensionados al peor caso (P: 5s @ 500us = 10000)
static const size_t MAX_SAMPLES = (CAPTURE_US_P / SAMPLE_US_P);

// ================== Buffers ==================
static uint16_t cs_mv_buf[MAX_SAMPLES];
static uint16_t vm_adc_mv_buf[MAX_SAMPLES];
static uint16_t ang_cdeg_buf[MAX_SAMPLES];   // centi-deg (0..36000)
static int32_t  omg_mrad_buf[MAX_SAMPLES];   // mrad/s (magnitud)

static volatile bool g_abort = false;
static bool g_capturing = false;

// ================== Driver helpers ==================
static inline void vnhForwardPins() {
  digitalWrite(SEL0, HIGH);
  digitalWrite(INA, HIGH);
  digitalWrite(INB, LOW);
}
static inline void vnhReversePins() {
  digitalWrite(SEL0, HIGH);
  digitalWrite(INA, LOW);
  digitalWrite(INB, HIGH);
}
static inline void vnhCoastPins() {
  digitalWrite(INA, LOW);
  digitalWrite(INB, LOW);
}
static inline void vnhStop() {
  pwmWriteTicks(0);
  vnhCoastPins();
}

static inline void applyMotorCmd(uint8_t cmd, uint8_t dutyPct,
                                 uint8_t &lastCmd, uint16_t &lastTicks) {
  uint16_t ticks = (cmd == MC_STOP) ? 0 : pctToTicks(dutyPct);

  if (cmd != lastCmd) {
    pwmWriteTicks(0);
    delayMicroseconds(50);

    if (cmd == MC_FWD)      vnhForwardPins();
    else if (cmd == MC_REV) vnhReversePins();
    else                   vnhCoastPins();

    lastCmd = cmd;
    lastTicks = 0;
  }

  if (ticks != lastTicks) {
    pwmWriteTicks(ticks);
    lastTicks = ticks;
  }
}

// runCapture parametrizable en muestreo/duración
static void runCapture(uint8_t dutyPct, uint8_t mode, uint32_t sample_us, uint32_t capture_us) {
  if (dutyPct > 100) dutyPct = 100;

  g_abort = false;
  g_capturing = true;

  // max samples para este modo (no siempre 10000)
  const size_t maxThis = (size_t)(capture_us / sample_us);
  const size_t maxN = (maxThis < MAX_SAMPLES) ? maxThis : MAX_SAMPLES;

  // --- Filtros (IIR) ---
  const float TAU_XY  = 0.003f;  // 3 ms para X,Y
  const float TAU_OMG = 0.010f;  // 10 ms para omega

  // Rechazo por magnitud de campo baja
  const float MAG2_ABS_MIN   = 200.0f * 200.0f;
  const float MAG2_REF_FRAC  = 0.05f;

  bool havePrev = false;
  float prevAngleDeg = 0.0f;

  bool haveXY = false;
  float xf = 0.0f, yf = 0.0f;
  float omegaFilt = 0.0f;
  float mag2_ref = 0.0f;

  const uint32_t t0 = micros();
  uint32_t next = t0;
  uint32_t prevSampleUs = t0;

  uint8_t  lastCmd   = MC_STOP;
  uint16_t lastTicks = 0;

  // Arranque: forward
  applyMotorCmd(MC_FWD, dutyPct, lastCmd, lastTicks);

  size_t n = 0;

  while (n < maxN) {
    // Abort rápido si llega 'S'
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == 'S' || c == 's') g_abort = true;
    }
    if (g_abort) break;

    uint32_t now = micros();
    uint32_t elapsed = now - t0;
    if (elapsed >= capture_us) break;

    if ((int32_t)(now - next) < 0) continue;
    next += sample_us;

    // ====== Programa P (SIEMPRE sensa, incluso en STOP) ======
    if (mode == CAP_PROGRAM_P) {
      // 0-1s: FWD, 1-2s: STOP, 2-3s: REV, 3-5s: STOP
      uint8_t desired =
        (elapsed < 1000000UL) ? MC_FWD :
        (elapsed < 2000000UL) ? MC_STOP :
        (elapsed < 3000000UL) ? MC_REV :
                                MC_STOP;

      applyMotorCmd(desired, dutyPct, lastCmd, lastTicks);
    } else {
      applyMotorCmd(MC_FWD, dutyPct, lastCmd, lastTicks);
    }

    // dt real
    float dt_s = (now - prevSampleUs) * 1e-6f;
    prevSampleUs = now;

    const float Ts = (float)sample_us * 1e-6f;
    if (dt_s < 0.2f * Ts) dt_s = Ts;
    if (dt_s > 5.0f * Ts) dt_s = Ts;

    // --- ADCs ---
    uint16_t vcs_mV  = (uint16_t)analogReadMilliVolts(CS_ADC);
    uint16_t vadc_mV = (uint16_t)analogReadMilliVolts(VMOTOR_ADC);

    // --- TMAG: X,Y ---
    int16_t x = (int16_t)tmagRead16(REG_X_CH_RESULT);
    int16_t y = (int16_t)tmagRead16(REG_Y_CH_RESULT);

    // --- Filtro IIR X,Y ---
    float alphaXY = dt_s / (TAU_XY + dt_s);
    if (!haveXY) {
      xf = (float)x; yf = (float)y;
      haveXY = true;
      mag2_ref = xf*xf + yf*yf;
      if (mag2_ref < MAG2_ABS_MIN) mag2_ref = MAG2_ABS_MIN;
    } else {
      xf += alphaXY * ((float)x - xf);
      yf += alphaXY * ((float)y - yf);
    }

    float mag2 = xf*xf + yf*yf;
    float mag2_min = MAG2_REF_FRAC * mag2_ref;
    if (mag2_min < MAG2_ABS_MIN) mag2_min = MAG2_ABS_MIN;

    // --- Ángulo + omega ---
    float angleDeg = prevAngleDeg;
    float omegaRad_signed = 0.0f;

    if (mag2 >= mag2_min) {
      angleDeg = angleDegXYf(xf, yf);

      if (havePrev) {
        float ddeg = unwrapDeltaPrevNowDeg(prevAngleDeg, angleDeg);
        omegaRad_signed = (ddeg * (3.1415926f / 180.0f)) / dt_s;
      } else {
        havePrev = true;
      }
      prevAngleDeg = angleDeg;
    } else {
      omegaRad_signed = 0.0f;
    }

    // --- Filtro IIR omega ---
    float alphaOmg = dt_s / (TAU_OMG + dt_s);
    omegaFilt += alphaOmg * (omegaRad_signed - omegaFilt);
    float omegaOut = fabsf(omegaFilt);

    // --- Guarda ---
    cs_mv_buf[n]     = vcs_mV;
    vm_adc_mv_buf[n] = vadc_mV;

    int32_t acdeg = (int32_t)lroundf(angleDeg * 100.0f);
    if (acdeg < 0) acdeg = 0;
    if (acdeg > 36000) acdeg = 36000;
    ang_cdeg_buf[n] = (uint16_t)acdeg;

    omg_mrad_buf[n] = (int32_t)lroundf(omegaOut * 1000.0f);

    n++;
  }

  vnhStop();
  g_capturing = false;

  // ===== Output SOLO matriz =====
  // idx t_us Vcs_mV I_est_mA Vadc_motor_mV Vmotor_mV angle_deg omega_rad_s
  for (size_t i = 0; i < n; i++) {
    uint32_t t_us = (uint32_t)i * sample_us;

    uint16_t vcs_mV = cs_mv_buf[i];
    uint32_t i_mA   = mvToCurrent_mA(vcs_mV);

    uint16_t vadc_mV = vm_adc_mv_buf[i];
    uint32_t vm_mV   = adcMvToMotorMv(vadc_mV);

    float angleDeg = ang_cdeg_buf[i] / 100.0f;
    float omegaRad = omg_mrad_buf[i] / 1000.0f;

    Serial.print((unsigned)i);              Serial.print(' ');
    Serial.print((unsigned long)t_us);      Serial.print(' ');
    Serial.print((unsigned)vcs_mV);         Serial.print(' ');
    Serial.print((unsigned long)i_mA);      Serial.print(' ');
    Serial.print((unsigned)vadc_mV);        Serial.print(' ');
    Serial.print((unsigned long)vm_mV);     Serial.print(' ');
    Serial.print(angleDeg, 2);              Serial.print(' ');
    Serial.println(omegaRad, 4);
  }
}

// ================== Serial parser (MXX, S/STOP y P) ==================
static void handleLine(String s) {
  s.trim();
  if (!s.length()) return;

  if (s == "S" || s == "s" || s == "STOP" || s == "stop") {
    g_abort = true;
    vnhStop();
    return;
  }

  // P: rutina 5s a 50% (500us)
  if ((s == "P" || s == "p") && !g_capturing) {
    runCapture(50, CAP_PROGRAM_P, SAMPLE_US_P, CAPTURE_US_P);
    return;
  }

  // MXX: SOLO ESTE comando usa 250us y 2s
  if ((s[0] == 'M' || s[0] == 'm') && !g_capturing) {
    int pct = s.substring(1).toInt();
    pct = constrain(pct, 0, 100);
    runCapture((uint8_t)pct, CAP_CONST_FWD, SAMPLE_US_M, CAPTURE_US_M);
  }
}

void setup() {
  Serial.begin(115200);
  delay(150);

  // VNH pins
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(SEL0, OUTPUT);
  digitalWrite(SEL0, HIGH);
  vnhStop();
  pwmInit();

  // ADC config
  analogReadResolution(12);
  analogSetPinAttenuation(CS_ADC, ADC_11db);
  analogSetPinAttenuation(VMOTOR_ADC, ADC_11db);

  // TMAG SPI
  pinMode(PIN_CS, OUTPUT);
  csHigh();
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);

  disableCRC_viaTestConfig();
  configXYZ();
}

void loop() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length()) handleLine(line);
      line = "";
    } else {
      // Stop inmediato si llega 'S' suelta
      if (c == 'S' || c == 's') {
        g_abort = true;
        vnhStop();
        line = "";
      } else {
        line += c;
      }
    }
  }
  delay(1);
}