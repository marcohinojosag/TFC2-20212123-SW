#pragma once
#include <Arduino.h>

class VNH7070 {
public:
  struct Pins {
    int ina;
    int inb;
    int pwm;
    int sel0;   // -1 si SEL0 está fijo a 3.3V
    int csAdc;  // ADC pin para CS (voltaje en Rsense)
  };

  void begin(const Pins &pins,
             int pwmChannel = 0,
             int pwmFreq = 20000,
             int pwmResBits = 10)
  {
    _pins = pins;
    _pwmCh = pwmChannel;
    _pwmFreq = pwmFreq;
    _pwmRes = pwmResBits;

    pinMode(_pins.ina, OUTPUT);
    pinMode(_pins.inb, OUTPUT);
    if (_pins.sel0 >= 0) pinMode(_pins.sel0, OUTPUT);

    // ADC (mejor ADC1 en ESP32)
    analogSetPinAttenuation(_pins.csAdc, ADC_11db);

    // PWM LEDC
    ledcSetup(_pwmCh, _pwmFreq, _pwmRes);
    ledcAttachPin(_pins.pwm, _pwmCh);

    // Estado inicial
    sel0(true);
    setHiZ();
  }

  // Ajustes
  void setRsenseOhms(float rsense) { _rsense = rsense; }
  void setKtyp(float k) { _kTyp = k; }  // aproximación (luego calibramos)

  // Motor: un solo sentido
  void setForwardDuty(float duty01) {
    sel0(true);
    digitalWrite(_pins.ina, HIGH);
    digitalWrite(_pins.inb, LOW);
    pwmWrite(duty01);
  }

  void setHiZ() {
    pwmForceLow();
    digitalWrite(_pins.ina, LOW);
    digitalWrite(_pins.inb, LOW);
  }

  // Corriente (aprox): Iout ≈ K * (Vcs/Rsense)
  float readCurrent_A() const {
    float vSense = analogReadMilliVolts(_pins.csAdc) / 1000.0f; // V
    float iSense = vSense / _rsense;                            // A
    return _kTyp * iSense;
  }

  // Voltaje de sense (útil para debug)
  float readCsVoltage_V() const {
    return analogReadMilliVolts(_pins.csAdc) / 1000.0f;
  }

private:
  Pins _pins{};
  int _pwmCh = 0;
  int _pwmFreq = 20000;
  int _pwmRes = 10;

  float _rsense = 1000.0f; // tu Rsense
  float _kTyp = 1540.0f;   // típico en corrientes medias-altas (aprox)

  void sel0(bool high) const {
    if (_pins.sel0 >= 0) digitalWrite(_pins.sel0, high ? HIGH : LOW);
  }

  int maxDuty() const { return (1 << _pwmRes) - 1; }

  void pwmWrite(float duty01) const {
    duty01 = constrain(duty01, 0.0f, 1.0f);
    ledcWrite(_pwmCh, (int)lroundf(duty01 * maxDuty()));
  }

  void pwmForceLow() const { ledcWrite(_pwmCh, 0); }
};
