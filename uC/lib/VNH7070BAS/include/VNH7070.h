#pragma once
#include <Arduino.h>

namespace VNH7070 {

// ================== Pines ==================
// NOTE: GPIO4 reservado para interrupción E-STOP -> INB movido a GPIO27.
static constexpr uint8_t INA_PIN_U8      = 21U;
static constexpr uint8_t INB_PIN_U8      = 27U;
static constexpr uint8_t SEL0_PIN_U8     = 16U;
static constexpr uint8_t PWM_PIN_U8      = 17U;

static constexpr uint8_t CS_ADC_PIN_U8     = 34U;  // Corriente (ADC1)
static constexpr uint8_t VMOTOR_ADC_PIN_U8 = 35U;  // Voltaje motor (ADC1)

// ================== PWM ==================
static constexpr uint32_t PWM_FREQ_HZ_U32 = 20000UL;
static constexpr uint8_t  PWM_RES_BITS_U8 = 10U;
static constexpr uint8_t  PWM_CH_U8       = 0U;

static inline uint16_t pwmMaxDuty_u16() {
  return (uint16_t)((1UL << (uint32_t)PWM_RES_BITS_U8) - 1UL);
}

static inline uint16_t pctToTicks_u16(uint8_t pct_u8) {
  if (pct_u8 > 100U) { pct_u8 = 100U; }
  const uint32_t ticks_u32 =
    ((uint32_t)pct_u8 * (uint32_t)pwmMaxDuty_u16() + 50UL) / 100UL;
  return (uint16_t)ticks_u32;
}

static inline void pwmInit() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(PWM_PIN_U8, PWM_FREQ_HZ_U32, PWM_RES_BITS_U8);
#else
  ledcSetup((uint32_t)PWM_CH_U8, PWM_FREQ_HZ_U32, (uint32_t)PWM_RES_BITS_U8);
  ledcAttachPin(PWM_PIN_U8, (uint32_t)PWM_CH_U8);
#endif
}

static inline void pwmWriteTicks(uint16_t ticks_u16) {
  const uint16_t max_u16 = pwmMaxDuty_u16();
  if (ticks_u16 > max_u16) { ticks_u16 = max_u16; }

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(PWM_PIN_U8, (uint32_t)ticks_u16);
#else
  ledcWrite((uint32_t)PWM_CH_U8, (uint32_t)ticks_u16);
#endif
}

// ================== Conversión corriente ==================
static constexpr uint16_t RSENSE_OHM_U16 = 1000U;
static constexpr uint16_t K_TYP_U16      = 1540U;

static inline uint32_t mvToCurrent_mA_u32(uint16_t vcs_mV_u16) {
  return ((uint32_t)vcs_mV_u16 * (uint32_t)K_TYP_U16 + (uint32_t)(RSENSE_OHM_U16 / 2U))
         / (uint32_t)RSENSE_OHM_U16;
}

// ================== Divisor voltaje motor ==================
static constexpr uint16_t R_TOP_OHM_U16 = 2900U;
static constexpr uint16_t R_BOT_OHM_U16 = 970U;

static inline uint32_t adcMvToMotorMv_u32(uint16_t vadc_mV_u16) {
  const uint32_t num_u32 =
    (uint32_t)vadc_mV_u16 * (uint32_t)(R_TOP_OHM_U16 + R_BOT_OHM_U16);
  return (num_u32 + (uint32_t)(R_BOT_OHM_U16 / 2U)) / (uint32_t)R_BOT_OHM_U16;
}

// ================== Driver helpers ==================
static inline void forwardPins() {
  digitalWrite(SEL0_PIN_U8, HIGH);
  digitalWrite(INA_PIN_U8, HIGH);
  digitalWrite(INB_PIN_U8, LOW);
}

static inline void reversePins() {
  digitalWrite(SEL0_PIN_U8, HIGH);
  digitalWrite(INA_PIN_U8, LOW);
  digitalWrite(INB_PIN_U8, HIGH);
}

static inline void coastPins() {
  digitalWrite(INA_PIN_U8, LOW);
  digitalWrite(INB_PIN_U8, LOW);
}

static inline void stop() {
  pwmWriteTicks(0U);
  coastPins();
}

// ================== “Enums” como uint8_t ==================
static constexpr uint8_t MC_STOP_U8 = 0U;
static constexpr uint8_t MC_FWD_U8  = 1U;
static constexpr uint8_t MC_REV_U8  = 2U;

// Conmutación segura (deadtime + PWM=0 antes de invertir)
static inline void applyMotorCmd(uint8_t cmd_u8, uint8_t dutyPct_u8,
                                 uint8_t &lastCmd_u8, uint16_t &lastTicks_u16) {
  const uint16_t ticks_u16 = (cmd_u8 == MC_STOP_U8) ? 0U : pctToTicks_u16(dutyPct_u8);

  if (cmd_u8 != lastCmd_u8) {
    pwmWriteTicks(0U);
    delayMicroseconds(50U);

    if (cmd_u8 == MC_FWD_U8)      { forwardPins(); }
    else if (cmd_u8 == MC_REV_U8) { reversePins(); }
    else                          { coastPins(); }

    lastCmd_u8 = cmd_u8;
    lastTicks_u16 = 0U;
  }

  if (ticks_u16 != lastTicks_u16) {
    pwmWriteTicks(ticks_u16);
    lastTicks_u16 = ticks_u16;
  }
}

static inline void begin() {
  pinMode(INA_PIN_U8, OUTPUT);
  pinMode(INB_PIN_U8, OUTPUT);
  pinMode(SEL0_PIN_U8, OUTPUT);
  digitalWrite(SEL0_PIN_U8, HIGH);

  stop();
  pwmInit();

  analogReadResolution((uint8_t)12U);
  analogSetPinAttenuation(CS_ADC_PIN_U8, ADC_11db);
  analogSetPinAttenuation(VMOTOR_ADC_PIN_U8, ADC_11db);
}

} 