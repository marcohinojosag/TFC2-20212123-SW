#pragma once
#include <Arduino.h>
#include <SPI.h>

namespace TMAG5170 {

// ================== Pines SPI ==================
static constexpr uint8_t PIN_SCK_U8  = 18U;
static constexpr uint8_t PIN_MISO_U8 = 19U;
static constexpr uint8_t PIN_MOSI_U8 = 23U;
static constexpr uint8_t PIN_CS_U8   = 22U;

static constexpr int8_t SPI_SS_UNUSED_I8 = (int8_t)-1;

// ================== Registros (uint8_t) ==================
static constexpr uint8_t REG_DEVICE_CONFIG_U8 = 0x00U;
static constexpr uint8_t REG_SENSOR_CONFIG_U8 = 0x01U;
static constexpr uint8_t REG_SYSTEM_CONFIG_U8 = 0x02U;
static constexpr uint8_t REG_TEST_CONFIG_U8   = 0x0FU;

static constexpr uint8_t REG_X_CH_RESULT_U8   = 0x09U;
static constexpr uint8_t REG_Y_CH_RESULT_U8   = 0x0AU;

// ================== SPI settings ==================
static constexpr uint32_t SPI_HZ_U32      = 8000000UL;
static constexpr uint8_t  SPI_MODE_U8     = (uint8_t)SPI_MODE0;

static inline void csLow()  { digitalWrite(PIN_CS_U8, LOW); }
static inline void csHigh() { digitalWrite(PIN_CS_U8, HIGH); }

static inline uint16_t read16_u16(uint8_t addr_u8) {
  SPI.beginTransaction(SPISettings(SPI_HZ_U32, MSBFIRST, SPI_MODE_U8));
  csLow();

  SPI.transfer((uint8_t)(addr_u8 | 0x80U));
  const uint8_t b1_u8 = SPI.transfer(0x00U);
  const uint8_t b2_u8 = SPI.transfer(0x00U);
  (void)SPI.transfer(0x00U); // dummy (CRC off)

  csHigh();
  SPI.endTransaction();

  return (uint16_t)(((uint16_t)b1_u8 << 8U) | (uint16_t)b2_u8);
}

static inline void write16(uint8_t addr_u8, uint16_t data_u16, uint8_t lastByte_u8 = 0x00U) {
  SPI.beginTransaction(SPISettings(SPI_HZ_U32, MSBFIRST, SPI_MODE_U8));
  csLow();

  SPI.transfer((uint8_t)(addr_u8 & 0x7FU));
  SPI.transfer((uint8_t)((data_u16 >> 8U) & 0xFFU));
  SPI.transfer((uint8_t)((data_u16 >> 0U) & 0xFFU));
  SPI.transfer(lastByte_u8);

  csHigh();
  SPI.endTransaction();
  delayMicroseconds(5U);
}

// Deshabilitar CRC (0x0F000407)
static inline void disableCRC_viaTestConfig() {
  SPI.beginTransaction(SPISettings(SPI_HZ_U32, MSBFIRST, SPI_MODE_U8));
  csLow();
  SPI.transfer(REG_TEST_CONFIG_U8);
  SPI.transfer(0x00U);
  SPI.transfer(0x04U);
  SPI.transfer(0x07U);
  csHigh();
  SPI.endTransaction();
  delay(5U);
}

// Config: Active measure + habilitar XYZ + rangos ±100mT (0x41EA)
static inline void configXYZ() {
  write16(REG_DEVICE_CONFIG_U8, 0x0020U);
  write16(REG_SENSOR_CONFIG_U8, 0x41EAU);
  write16(REG_SYSTEM_CONFIG_U8, 0x0000U);
  delay(10U);
}

static inline void begin() {
  pinMode(PIN_CS_U8, OUTPUT);
  csHigh();
  SPI.begin(PIN_SCK_U8, PIN_MISO_U8, PIN_MOSI_U8, SPI_SS_UNUSED_I8);

  disableCRC_viaTestConfig();
  configXYZ();
}

static inline void readXY_i16(int16_t &x_i16, int16_t &y_i16) {
  x_i16 = (int16_t)read16_u16(REG_X_CH_RESULT_U8);
  y_i16 = (int16_t)read16_u16(REG_Y_CH_RESULT_U8);
}

} 