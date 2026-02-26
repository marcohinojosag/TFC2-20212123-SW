#include <Arduino.h>
#include <math.h>
#include "VNH7070.h"
#include "TMAG5170.h"

// ================== Timings por comando ==================
static constexpr uint32_t SAMPLE_US_M_U32  = 250UL;
static constexpr uint32_t CAPTURE_US_M_U32 = 2000000UL;

static constexpr uint32_t SAMPLE_US_P_U32  = 500UL;
static constexpr uint32_t CAPTURE_US_P_U32 = 5000000UL;

static constexpr uint8_t CAP_CONST_FWD_U8 = 0U;
static constexpr uint8_t CAP_PROGRAM_P_U8 = 1U;

static constexpr size_t MAX_SAMPLES = (size_t)(CAPTURE_US_P_U32 / SAMPLE_US_P_U32);

// ================== Relés + E-STOP ==================
static constexpr uint8_t RELAY_RUN_PIN_U8   = 15U; // parpadea durante rutinas
static constexpr uint8_t RELAY_ESTOP_PIN_U8 = 26U; // se activa 5s por interrupción
static constexpr uint8_t ESTOP_SW_PIN_U8    = 4U;  // interrupción (INPUT_PULLUP)

static constexpr uint8_t RELAY_ON_U8  = (uint8_t)HIGH;
static constexpr uint8_t RELAY_OFF_U8 = (uint8_t)LOW;

static constexpr uint32_t RUN_BLINK_HALF_MS_U32 = 500UL;   // 0.5s ON / 0.5s OFF
static constexpr uint32_t ESTOP_HOLD_MS_U32     = 5000UL;  // 5s

static volatile bool g_abort = false;
static bool g_capturing = false;

// IRQ flags
static volatile bool g_estop_irq = false;

// Relay states/timers
static uint32_t g_estop_until_ms_u32 = 0UL;
static uint32_t g_last_blink_ms_u32  = 0UL;
static bool g_run_state = false;

static inline void relayWrite(uint8_t pin_u8, bool on_b) {
  digitalWrite(pin_u8, on_b ? RELAY_ON_U8 : RELAY_OFF_U8);
}

static inline void serviceRelays(uint32_t now_ms_u32, bool running_b) {
  // Si hubo interrupción, activar relay ESTOP por 5s (en contexto NO-ISR)
  if (g_estop_irq) {
    g_estop_irq = false;
    g_estop_until_ms_u32 = now_ms_u32 + ESTOP_HOLD_MS_U32;
  }

  // Relay ESTOP (activo si now < until) con wrap-safe
  const bool estop_active_b = ((int32_t)(now_ms_u32 - g_estop_until_ms_u32) < 0);
  relayWrite(RELAY_ESTOP_PIN_U8, estop_active_b);

  // Relay RUN (blink solo mientras corre una rutina)
  if (!running_b) {
    g_run_state = false;
    relayWrite(RELAY_RUN_PIN_U8, false);
    g_last_blink_ms_u32 = now_ms_u32;
    return;
  }

  if ((uint32_t)(now_ms_u32 - g_last_blink_ms_u32) >= RUN_BLINK_HALF_MS_U32) {
    g_last_blink_ms_u32 = now_ms_u32;
    g_run_state = !g_run_state;
    relayWrite(RELAY_RUN_PIN_U8, g_run_state);
  }
}

// ISR E-STOP
static void IRAM_ATTR onEstopISR() {
  g_estop_irq = true;
  g_abort = true;
}

// ================== Buffers ==================
static uint16_t cs_mv_buf[MAX_SAMPLES];
static uint16_t vm_adc_mv_buf[MAX_SAMPLES];
static uint16_t ang_cdeg_buf[MAX_SAMPLES];
static int32_t  omg_mrad_buf[MAX_SAMPLES];

static inline float angleDegXYf(float x_f, float y_f) {
  float a_f = atan2f(y_f, x_f) * (180.0f / 3.1415926f);
  if (a_f < 0.0f) { a_f += 360.0f; }
  return a_f;
}

static inline float unwrapDeltaPrevNowDeg(float aPrev_f, float aNow_f) {
  float d_f = aPrev_f - aNow_f;
  if (d_f > 180.0f) { d_f -= 360.0f; }
  if (d_f < -180.0f) { d_f += 360.0f; }
  return d_f;
}

static void runCapture(uint8_t dutyPct_u8, uint8_t mode_u8, uint32_t sample_us_u32, uint32_t capture_us_u32) {
  if (dutyPct_u8 > 100U) { dutyPct_u8 = 100U; }

  g_abort = false;
  g_capturing = true;

  const size_t maxThis = (size_t)(capture_us_u32 / sample_us_u32);
  const size_t maxN = (maxThis < MAX_SAMPLES) ? maxThis : MAX_SAMPLES;

  // indicador RUN (arranque)
  const uint32_t start_ms_u32 = millis();
  g_last_blink_ms_u32 = start_ms_u32;
  g_run_state = false;
  relayWrite(RELAY_RUN_PIN_U8, false);

  // Filtros
  const float TAU_XY_f  = 0.003f;
  const float TAU_OMG_f = 0.010f;

  const float MAG2_ABS_MIN_f  = 200.0f * 200.0f;
  const float MAG2_REF_FRAC_f = 0.05f;

  bool havePrev = false;
  float prevAngleDeg_f = 0.0f;

  bool haveXY = false;
  float xf = 0.0f;
  float yf = 0.0f;
  float omegaFilt_f = 0.0f;
  float mag2_ref_f = 0.0f;

  const uint32_t t0_u32 = micros();
  uint32_t next_u32 = t0_u32;
  uint32_t prevSampleUs_u32 = t0_u32;

  uint8_t  lastCmd_u8   = VNH7070::MC_STOP_U8;
  uint16_t lastTicks_u16 = 0U;

  VNH7070::applyMotorCmd(VNH7070::MC_FWD_U8, dutyPct_u8, lastCmd_u8, lastTicks_u16);

  size_t n = 0U;

  while (n < maxN) {
    const uint32_t now_ms_u32 = millis();
    serviceRelays(now_ms_u32, true);

    // stop por interrupción
    if (g_abort) {
      VNH7070::stop();
      break;
    }

    // Abort por Serial 'S'
    while (Serial.available()) {
      const char c_ch = (char)Serial.read();
      if ((c_ch == 'S') || (c_ch == 's')) { g_abort = true; }
    }
    if (g_abort) {
      VNH7070::stop();
      break;
    }

    const uint32_t now_u32 = micros();
    const uint32_t elapsed_u32 = now_u32 - t0_u32;
    if (elapsed_u32 >= capture_us_u32) { break; }

    if ((int32_t)(now_u32 - next_u32) < 0) { continue; }
    next_u32 += sample_us_u32;

    // Programa P
    if (mode_u8 == CAP_PROGRAM_P_U8) {
      const uint8_t desired_u8 =
        (elapsed_u32 < 1000000UL) ? VNH7070::MC_FWD_U8 :
        (elapsed_u32 < 2000000UL) ? VNH7070::MC_STOP_U8 :
        (elapsed_u32 < 3000000UL) ? VNH7070::MC_REV_U8 :
                                    VNH7070::MC_STOP_U8;

      VNH7070::applyMotorCmd(desired_u8, dutyPct_u8, lastCmd_u8, lastTicks_u16);
    } else {
      VNH7070::applyMotorCmd(VNH7070::MC_FWD_U8, dutyPct_u8, lastCmd_u8, lastTicks_u16);
    }

    // dt real
    float dt_s_f = (float)(now_u32 - prevSampleUs_u32) * 1e-6f;
    prevSampleUs_u32 = now_u32;

    const float Ts_f = (float)sample_us_u32 * 1e-6f;
    if (dt_s_f < 0.2f * Ts_f) { dt_s_f = Ts_f; }
    if (dt_s_f > 5.0f * Ts_f) { dt_s_f = Ts_f; }

    // ADCs
    const uint16_t vcs_mV_u16  = (uint16_t)analogReadMilliVolts(VNH7070::CS_ADC_PIN_U8);
    const uint16_t vadc_mV_u16 = (uint16_t)analogReadMilliVolts(VNH7070::VMOTOR_ADC_PIN_U8);

    // TMAG XY
    int16_t x_i16 = (int16_t)0;
    int16_t y_i16 = (int16_t)0;
    TMAG5170::readXY_i16(x_i16, y_i16);

    // IIR XY
    const float alphaXY_f = dt_s_f / (TAU_XY_f + dt_s_f);
    if (!haveXY) {
      xf = (float)x_i16;
      yf = (float)y_i16;
      haveXY = true;
      mag2_ref_f = xf * xf + yf * yf;
      if (mag2_ref_f < MAG2_ABS_MIN_f) { mag2_ref_f = MAG2_ABS_MIN_f; }
    } else {
      xf += alphaXY_f * ((float)x_i16 - xf);
      yf += alphaXY_f * ((float)y_i16 - yf);
    }

    const float mag2_f = xf * xf + yf * yf;
    float mag2_min_f = MAG2_REF_FRAC_f * mag2_ref_f;
    if (mag2_min_f < MAG2_ABS_MIN_f) { mag2_min_f = MAG2_ABS_MIN_f; }

    // Ángulo + omega
    float angleDeg_f = prevAngleDeg_f;
    float omegaRad_signed_f = 0.0f;

    if (mag2_f >= mag2_min_f) {
      angleDeg_f = angleDegXYf(xf, yf);

      if (havePrev) {
        const float ddeg_f = unwrapDeltaPrevNowDeg(prevAngleDeg_f, angleDeg_f);
        omegaRad_signed_f = (ddeg_f * (3.1415926f / 180.0f)) / dt_s_f;
      } else {
        havePrev = true;
      }
      prevAngleDeg_f = angleDeg_f;
    } else {
      omegaRad_signed_f = 0.0f;
    }

    // IIR omega
    const float alphaOmg_f = dt_s_f / (TAU_OMG_f + dt_s_f);
    omegaFilt_f += alphaOmg_f * (omegaRad_signed_f - omegaFilt_f);
    const float omegaOut_f = fabsf(omegaFilt_f);

    // Guarda
    cs_mv_buf[n]     = vcs_mV_u16;
    vm_adc_mv_buf[n] = vadc_mV_u16;

    int32_t acdeg_i32 = (int32_t)lroundf(angleDeg_f * 100.0f);
    if (acdeg_i32 < 0) { acdeg_i32 = 0; }
    if (acdeg_i32 > 36000) { acdeg_i32 = 36000; }
    ang_cdeg_buf[n] = (uint16_t)acdeg_i32;

    omg_mrad_buf[n] = (int32_t)lroundf(omegaOut_f * 1000.0f);

    n++;
  }

  VNH7070::stop();

  // Output matriz (mantengo RUN blink mientras imprime)
  for (size_t i = 0U; i < n; i++) {
    serviceRelays(millis(), true);

    const uint32_t t_us_u32 = (uint32_t)i * sample_us_u32;

    const uint16_t vcs_mV_u16 = cs_mv_buf[i];
    const uint32_t i_mA_u32   = VNH7070::mvToCurrent_mA_u32(vcs_mV_u16);

    const uint16_t vadc_mV_u16 = vm_adc_mv_buf[i];
    const uint32_t vm_mV_u32   = VNH7070::adcMvToMotorMv_u32(vadc_mV_u16);

    const float angleDeg_f = (float)ang_cdeg_buf[i] / 100.0f;
    const float omegaRad_f = (float)omg_mrad_buf[i] / 1000.0f;

    Serial.print((uint32_t)i);                 Serial.print(' ');
    Serial.print((unsigned long)t_us_u32);     Serial.print(' ');
    Serial.print((uint32_t)vcs_mV_u16);        Serial.print(' ');
    Serial.print((unsigned long)i_mA_u32);     Serial.print(' ');
    Serial.print((uint32_t)vadc_mV_u16);       Serial.print(' ');
    Serial.print((unsigned long)vm_mV_u32);    Serial.print(' ');
    Serial.print(angleDeg_f, 2);               Serial.print(' ');
    Serial.println(omegaRad_f, 4);
  }

  g_capturing = false;
  serviceRelays(millis(), false);
}

// ================== Serial parser (MXX, S/STOP y P) ==================
static void handleLine(String s) {
  s.trim();
  if (!s.length()) { return; }

  if ((s == "S") || (s == "s") || (s == "STOP") || (s == "stop")) {
    g_abort = true;
    VNH7070::stop();
    return;
  }

  if (((s == "P") || (s == "p")) && (!g_capturing)) {
    runCapture(50U, CAP_PROGRAM_P_U8, SAMPLE_US_P_U32, CAPTURE_US_P_U32);
    return;
  }

  if ((((char)s[0] == 'M') || ((char)s[0] == 'm')) && (!g_capturing)) {
    int32_t pct_i32 = (int32_t)s.substring(1).toInt();
    if (pct_i32 < 0) { pct_i32 = 0; }
    if (pct_i32 > 100) { pct_i32 = 100; }
    runCapture((uint8_t)pct_i32, CAP_CONST_FWD_U8, SAMPLE_US_M_U32, CAPTURE_US_M_U32);
  }
}

void setup() {
  Serial.begin(115200U);
  delay(150U);

  // Relés
  pinMode(RELAY_RUN_PIN_U8, OUTPUT);
  pinMode(RELAY_ESTOP_PIN_U8, OUTPUT);
  relayWrite(RELAY_RUN_PIN_U8, false);
  relayWrite(RELAY_ESTOP_PIN_U8, false);

  // E-STOP switch (asumo a GND con pullup interno)
  pinMode(ESTOP_SW_PIN_U8, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ESTOP_SW_PIN_U8), onEstopISR, FALLING);

  VNH7070::begin();
  TMAG5170::begin();
}

void loop() {
  // Mantener relés en reposo (ESTOP hold si aplica)
  serviceRelays(millis(), g_capturing);

  static String line;
  while (Serial.available()) {
    const char c_ch = (char)Serial.read();
    if ((c_ch == '\n') || (c_ch == '\r')) {
      if (line.length()) { handleLine(line); }
      line = "";
    } else {
      if ((c_ch == 'S') || (c_ch == 's')) {
        g_abort = true;
        VNH7070::stop();
        line = "";
      } else {
        line += c_ch;
      }
    }
  }
  delay(1U);
}