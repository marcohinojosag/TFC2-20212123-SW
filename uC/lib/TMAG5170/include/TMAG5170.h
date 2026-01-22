#pragma once
#include <Arduino.h>
#include <SPI.h>

class TMAG5170SPI {
public:
  struct Pins { int cs; };

  void begin(SPIClass &spi, const Pins &pins, uint32_t hz = 1000000, uint8_t spiMode = SPI_MODE0) {
    _spi = &spi;
    _pins = pins;
    _hz = hz;
    _mode = spiMode;

    pinMode(_pins.cs, OUTPUT);
    digitalWrite(_pins.cs, HIGH);
  }

  // -------- Configs --------
  void configAngleXY_A1() {
    writeReg(REG_DEVICE_CONFIG, 0x0020);
    writeReg(REG_SENSOR_CONFIG, 0x40C5);
    writeReg(REG_SYSTEM_CONFIG, 0x0000);
    delay(5);
  }

  float readAngleDeg() {
    uint16_t w = readReg(REG_ANGLE_RESULT);
    uint16_t code = (w & 0x1FFF);
    return (float)code / 16.0f;
  }

  uint16_t readReg(uint8_t addr) {
    uint32_t frame = buildFrame(true, addr, 0x0000, 0x0);
    uint32_t rx = transfer32(frame);
    return (uint16_t)((rx >> 8) & 0xFFFF);
  }

  void writeReg(uint8_t addr, uint16_t value) {
    uint32_t frame = buildFrame(false, addr, value, 0x0);
    (void)transfer32(frame);
  }

  static constexpr uint8_t REG_DEVICE_CONFIG = 0x00;
  static constexpr uint8_t REG_SENSOR_CONFIG = 0x01;
  static constexpr uint8_t REG_SYSTEM_CONFIG = 0x02;
  static constexpr uint8_t REG_ANGLE_RESULT  = 0x13;

private:
  SPIClass *_spi = nullptr;
  Pins _pins{};
  uint32_t _hz = 1000000;
  uint8_t _mode = SPI_MODE0;

  static uint8_t crc4(uint32_t frame_crc0) {
    uint8_t crc = 0xF;
    for (int i = 31; i >= 0; --i) {
      uint8_t bit = (frame_crc0 >> i) & 1;
      uint8_t inv = bit ^ ((crc >> 3) & 1);

      uint8_t c3 = (crc >> 2) & 1;
      uint8_t c2 = (crc >> 1) & 1;
      uint8_t c1 = (crc >> 0) & 1;

      uint8_t n3 = c3;
      uint8_t n2 = c2;
      uint8_t n1 = c1 ^ inv;
      uint8_t n0 = inv;

      crc = (n3 << 3) | (n2 << 2) | (n1 << 1) | (n0 << 0);
    }
    return crc & 0x0F;
  }

  uint32_t buildFrame(bool isRead, uint8_t addr, uint16_t data, uint8_t cmdNibble) {
    uint32_t f = 0;
    f |= (isRead ? 1UL : 0UL) << 31;
    f |= (uint32_t)(addr & 0x7F) << 24;
    f |= (uint32_t)data << 8;
    f |= (uint32_t)(cmdNibble & 0x0F) << 4;

    uint32_t f0 = (f & 0xFFFFFFF0);
    f0 |= (uint32_t)(crc4(f0) & 0x0F);
    return f0;
  }

  uint32_t transfer32(uint32_t tx) {
    _spi->beginTransaction(SPISettings(_hz, MSBFIRST, _mode));
    digitalWrite(_pins.cs, LOW);

    uint8_t b3 = (tx >> 24) & 0xFF;
    uint8_t b2 = (tx >> 16) & 0xFF;
    uint8_t b1 = (tx >>  8) & 0xFF;
    uint8_t b0 = (tx >>  0) & 0xFF;

    uint32_t rx = 0;
    rx |= (uint32_t)_spi->transfer(b3) << 24;
    rx |= (uint32_t)_spi->transfer(b2) << 16;
    rx |= (uint32_t)_spi->transfer(b1) <<  8;
    rx |= (uint32_t)_spi->transfer(b0) <<  0;

    digitalWrite(_pins.cs, HIGH);
    _spi->endTransaction();
    return rx;
  }
};
