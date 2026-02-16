#include <Arduino.h>
#include "esp32-hal-adc.h"

// ===== Pines VNH7070 =====
static const int INA     = 21;
static const int INB     = 4;
static const int SEL0    = 16;
static const int PWM_PIN = 17;
static const int CS_ADC  = 34;   // ADC1

// ===== PWM =====
static const int PWM_FREQ = 20000;
static const int PWM_RES  = 10;
static inline uint16_t pwmMaxDuty() { return (1u << PWM_RES) - 1u; }

static void pwmInit() {
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);   // core 3.x
}
static void pwmWriteTicks(uint16_t ticks) {
  if (ticks > pwmMaxDuty()) ticks = pwmMaxDuty();
  ledcWrite(PWM_PIN, ticks);                // core 3.x
}
static inline uint16_t pctToTicks(uint8_t pct) {
  if (pct > 100) pct = 100;
  return (uint16_t)(((uint32_t)pct * (uint32_t)pwmMaxDuty() + 50u) / 100u);
}

static inline void vnhForward() {
  digitalWrite(SEL0, HIGH);
  digitalWrite(INA, HIGH);
  digitalWrite(INB, LOW);
}
static inline void vnhStop() {
  pwmWriteTicks(0);
  digitalWrite(INA, LOW);
  digitalWrite(INB, LOW);
}

// ===== Captura ADC continuo =====
static const uint32_t CAPTURE_US = 2000000UL;  // 2 s
static const uint32_t SAMPLE_HZ  = 10000UL;    // 10 kHz -> 100 us
static const size_t   MAX_SAMPLES = (CAPTURE_US * SAMPLE_HZ) / 1000000UL; // 20000

static uint16_t rawBuf[MAX_SAMPLES];
static uint16_t mvBuf [MAX_SAMPLES];

static const uint16_t SCALE_1540 = 1540;

static volatile bool g_abort = false;

// ===== Parser súper simple: dispara en MXX sin depender de enter =====
static int  g_mDigits = 0;
static int  g_mValue  = 0;

static inline bool isDigitChar(char c) { return (c >= '0' && c <= '9'); }

static void feedCmdChar(char c) {
  if (c == 'S' || c == 's') {
    g_abort = true;
    vnhStop();
    g_mDigits = 0;
    g_mValue = 0;
    return;
  }

  if (c == 'M' || c == 'm') {
    g_mDigits = 0;
    g_mValue  = 0;
    return;
  }

  if (g_mDigits >= 0 && isDigitChar(c)) {
    if (g_mDigits < 3) {
      g_mValue = g_mValue * 10 + (c - '0');
      g_mDigits++;
    }
    return;
  }

  // cualquier otro char resetea si estabas a medias
  if (g_mDigits > 0) {
    // no hacemos nada aquí: el disparo será al llegar a 2 dígitos
  }
}

static void runCapture(uint8_t dutyPct) {
  g_abort = false;

  // Arranca motor (para que SIEMPRE se mueva aunque algo falle luego)
  vnhForward();
  pwmWriteTicks(pctToTicks(dutyPct));

  // ADC continuous setup
  uint8_t pins[] = { (uint8_t)CS_ADC };
  analogContinuousSetWidth(12);
  analogContinuousSetAtten(ADC_11db);

  if (!analogContinuous(pins, 1, 1, SAMPLE_HZ, NULL)) {
    // fallback: si no inicia continuous, igual detiene motor
    vnhStop();
    return;
  }
  if (!analogContinuousStart()) {
    analogContinuousDeinit();
    vnhStop();
    return;
  }

  size_t n = 0;
  uint32_t t0 = micros();

  while (n < MAX_SAMPLES && (micros() - t0) < CAPTURE_US && !g_abort) {
    // permite abortar con 'S' durante captura
    while (Serial.available()) {
      char c = (char)Serial.read();
      feedCmdChar(c);  // aquí detecta 'S'
    }
    if (g_abort) break;

    adc_continuous_data_t *res = nullptr;
    if (analogContinuousRead(&res, 0) && res != nullptr) {
      rawBuf[n] = (uint16_t)res[0].avg_read_raw;
      mvBuf[n]  = (uint16_t)res[0].avg_read_mvolts;
      n++;
    }
  }

  analogContinuousStop();
  analogContinuousDeinit();
  vnhStop();

  // Dump al final: SOLO números
  // t_us raw mv mvx1540
  const uint32_t Ts_us = 1000000UL / SAMPLE_HZ; // 100 us
  for (size_t i = 0; i < n; i++) {
    uint32_t t_us = (uint32_t)(i * Ts_us);
    uint32_t mvx  = (uint32_t)mvBuf[i] * (uint32_t)SCALE_1540;
    Serial.printf("%lu %u %u %lu\n",
                  (unsigned long)t_us,
                  (unsigned)rawBuf[i],
                  (unsigned)mvBuf[i],
                  (unsigned long)mvx);
  }
}

void setup() {
  Serial.begin(115200);     // igual que tu flujo anterior
  delay(100);

  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(SEL0, OUTPUT);

  pwmInit();
  vnhStop();

  // ADC config base (no continuo)
  analogReadResolution(12);
  analogSetPinAttenuation(CS_ADC, ADC_11db);
}

void loop() {
  // Lee chars y ejecuta cuando ve MXX
  while (Serial.available()) {
    char c = (char)Serial.read();
    feedCmdChar(c);

    // dispara apenas tenga 2 dígitos (MXX)
    if (g_mDigits == 2) {
      int pct = g_mValue;
      g_mDigits = 0;
      g_mValue  = 0;

      if (pct < 0) pct = 0;
      if (pct > 100) pct = 100;
      runCapture((uint8_t)pct);
    }
  }
  delay(1);
}
