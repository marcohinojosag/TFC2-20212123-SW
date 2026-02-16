#include <Arduino.h>

// ===== Pines VNH7070 =====
static const int INA     = 21;
static const int INB     = 4;
static const int SEL0    = 16;
static const int PWM_PIN = 17;

// Corriente VNH (CS) en ADC:
static const int CS_ADC  = 34;   // ADC1

// Voltaje motor (divisor) en ADC:  <-- CAMBIA ESTE PIN
static const int VMOTOR_ADC = 35; // Ejemplo: GPIO35 (ADC1)

// ===== PWM =====
static const int PWM_FREQ = 20000;
static const int PWM_RES  = 10;

static inline uint16_t pwmMaxDuty() { return (1u << PWM_RES) - 1u; }
static inline uint16_t pctToTicks(uint8_t pct) {
  if (pct > 100) pct = 100;
  return (uint16_t)(((uint32_t)pct * (uint32_t)pwmMaxDuty() + 50u) / 100u);
}

// ===== Corriente (igual que tu lógica) =====
static const uint16_t RSENSE_OHM = 1000; // tu Rsense
static const uint16_t K_TYP      = 1540; // escala que estás usando

static inline uint32_t mvToCurrent_mA(uint16_t vcs_mV) {
  // I_est_mA = (Vcs_mV * 1540) / 1000
  return ((uint32_t)vcs_mV * (uint32_t)K_TYP + (RSENSE_OHM / 2)) / (uint32_t)RSENSE_OHM;
}

// ===== Divisor de voltaje medido =====
// Asumimos: Rtop = 2900 (a motor), Rbot = 970 (a GND), ADC en el nodo medio.
static const uint16_t R_TOP_OHM = 2900;
static const uint16_t R_BOT_OHM = 970;

static inline uint32_t adcMvToMotorMv(uint16_t vadc_mV) {
  // Vmotor_mV = Vadc_mV * (Rtop+Rbot) / Rbot
  const uint32_t num = (uint32_t)vadc_mV * (uint32_t)(R_TOP_OHM + R_BOT_OHM);
  return (num + (R_BOT_OHM / 2)) / (uint32_t)R_BOT_OHM; // redondeo
}

// ===== Captura =====
static const uint32_t CAPTURE_US = 800000UL; // 1 s
static const uint32_t SAMPLE_US  = 100UL;     // 100 us => 10 kS/s
static const size_t   MAX_SAMPLES = (CAPTURE_US / SAMPLE_US); // 10000

static uint16_t cs_mv_buf[MAX_SAMPLES];
static uint16_t vm_adc_mv_buf[MAX_SAMPLES];

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

static void runCapture(uint8_t dutyPct) {
  g_abort = false;
  g_capturing = true;

  vnhForward();
  ledcWrite(PWM_PIN, pctToTicks(dutyPct));

  const uint32_t t0 = micros();
  uint32_t next = t0;
  size_t n = 0;

  while (n < MAX_SAMPLES) {
    // Abort inmediato con 'S'
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

    cs_mv_buf[n]       = (uint16_t)analogReadMilliVolts(CS_ADC);
    vm_adc_mv_buf[n]   = (uint16_t)analogReadMilliVolts(VMOTOR_ADC);
    n++;
  }

  vnhStop();
  g_capturing = false;

  // Output SOLO NUMEROS:
  // idx t_us Vcs_mV I_est_mA Vadc_motor_mV Vmotor_mV
  for (size_t i = 0; i < n; i++) {
    uint32_t t_us   = (uint32_t)(i * SAMPLE_US);
    uint32_t i_mA   = mvToCurrent_mA(cs_mv_buf[i]);
    uint32_t vm_mV  = adcMvToMotorMv(vm_adc_mv_buf[i]);

    Serial.print((unsigned)i);                 Serial.print(' ');
    Serial.print((unsigned long)t_us);         Serial.print(' ');
    Serial.print((unsigned)cs_mv_buf[i]);      Serial.print(' ');
    Serial.print((unsigned long)i_mA);         Serial.print(' ');
    Serial.print((unsigned)vm_adc_mv_buf[i]);  Serial.print(' ');
    Serial.println((unsigned long)vm_mV);
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
  analogSetPinAttenuation(VMOTOR_ADC, ADC_11db);

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
      // Si llega 'S' suelta, aborta al toque
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
