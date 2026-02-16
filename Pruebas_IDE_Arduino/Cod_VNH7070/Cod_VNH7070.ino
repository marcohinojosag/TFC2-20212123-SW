#include <Arduino.h>

// ===== Pines VNH7070 =====
static const int INA     = 21;
static const int INB     = 4;
static const int SEL0    = 16;
static const int PWM_PIN = 17;
static const int CS_ADC  = 34;

// ===== PWM =====
static const int PWM_FREQ = 20000;
static const int PWM_RES  = 10;
static inline uint16_t pwmMaxDuty() { return (1u << PWM_RES) - 1u; }
static inline uint16_t pctToTicks(uint8_t pct) {
  if (pct > 100) pct = 100;
  return (uint16_t)(((uint32_t)pct * (uint32_t)pwmMaxDuty() + 50u) / 100u);
}

// ===== Escala corriente =====
static const uint16_t RSENSE_OHM = 1000;
static const uint16_t K_TYP      = 1540;
static inline uint32_t mvToCurrent_mA(uint16_t mv) {
  return ((uint32_t)mv * (uint32_t)K_TYP + (RSENSE_OHM / 2)) / (uint32_t)RSENSE_OHM;
}

// ===== Captura =====
static const uint32_t CAPTURE_US = 800000UL; // 1 s
static const uint32_t SAMPLE_US  = 100UL;     // 100 us => 10 kS/s
static const size_t   MAX_SAMPLES = (CAPTURE_US / SAMPLE_US); // 20000

// Solo guardamos mV (40 KB)
static uint16_t mvbuf[MAX_SAMPLES];

static volatile bool g_abort = false;
static bool g_capturing = false;

// ===== Driver =====
static inline void vnhForward() {
  digitalWrite(SEL0, HIGH);
  digitalWrite(INA, HIGH);
  digitalWrite(INB, LOW);
}
static inline void vnhStop() {
  ledcWrite(PWM_PIN, 0);
  digitalWrite(INA, LOW);
  digitalWrite(INB, LOW);
}

// ===== Captura y dump =====
static void runCapture(uint8_t dutyPct) {
  g_abort = false;
  g_capturing = true;

  vnhForward();
  ledcWrite(PWM_PIN, pctToTicks(dutyPct));

  const uint32_t t0 = micros();
  uint32_t next = t0;
  size_t n = 0;

  while (n < MAX_SAMPLES) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == 'S' || c == 's') g_abort = true;
    }
    if (g_abort) break;

    uint32_t now = micros();
    uint32_t dt  = now - t0;
    if (dt >= CAPTURE_US) break;
    if ((int32_t)(now - next) < 0) continue;
    next += SAMPLE_US;

    mvbuf[n] = (uint16_t)analogReadMilliVolts(CS_ADC);
    n++;
  }

  vnhStop();
  g_capturing = false;

  // Output SOLO numeros:
  // idx t_us Vcs_mV I_est_mA
  for (size_t i = 0; i < n; i++) {
    uint32_t t_us = (uint32_t)(i * SAMPLE_US);
    uint32_t imA  = mvToCurrent_mA(mvbuf[i]);
    Serial.print((unsigned)i);    Serial.print(' ');
    Serial.print((unsigned long)t_us); Serial.print(' ');
    Serial.print((unsigned)mvbuf[i]);  Serial.print(' ');
    Serial.println((unsigned long)imA);
  }
}

// ===== Parser: solo MXX y S =====
static void handleLine(String s) {
  s.trim();
  if (!s.length()) return;

  if (s == "S" || s == "s") {
    g_abort = true;
    vnhStop();
    return;
  }

  if ((s[0] == 'M' || s[0] == 'm') && !g_capturing) {
    int pct = s.substring(1).toInt();
    pct = constrain(pct, 0, 100);
    runCapture((uint8_t)pct);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(SEL0, OUTPUT);

  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);

  analogReadResolution(12);
  analogSetPinAttenuation(CS_ADC, ADC_11db);

  digitalWrite(SEL0, HIGH);
  vnhStop();
}

void loop() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length()) handleLine(line);
      line = "";
    } else {
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
