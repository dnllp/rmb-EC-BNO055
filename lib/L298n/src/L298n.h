#pragma once
// Un canal del puente H L298N: PWM (LEDC) en ENA/ENB y sentido con IN1/IN2 (o IN3/IN4).
//
//   EN   IN1 IN2   resultado
//   PWM   1   0    adelante
//   PWM   0   1    atrás
//   1     1   1    freno activo (los dos lados altos conducen)
//   0     x   x    rueda libre (desactivado)
//
// Quita el puente (jumper) de ENA/ENB del módulo: si se queda puesto, EN está siempre en alto
// y el PWM de la ESP32 no tiene efecto.
// El duty se limita a maxDuty para no pasar de ~6 V en el motor (ver roomba_config.h).
#include <Arduino.h>

class L298n {
 public:
  struct Pins {
    int en, in1, in2;
    uint8_t ch;  // canal LEDC
  };

  void begin(const Pins &p, uint32_t freqHz, uint8_t bits, bool invert, float maxDuty) {
    p_ = p;
    invert_ = invert;
    maxDuty_ = maxDuty > 1.0f ? 1.0f : (maxDuty < 0.1f ? 0.1f : maxDuty);
    fullScale_ = (1u << bits) - 1;
    pinMode(p_.in1, OUTPUT);
    pinMode(p_.in2, OUTPUT);
    digitalWrite(p_.in1, LOW);
    digitalWrite(p_.in2, LOW);
    ledcSetup(p_.ch, freqHz, bits);
    ledcAttachPin(p_.en, p_.ch);
    ledcWrite(p_.ch, 0);
    enabled_ = false;
  }

  // Sin enable() el canal queda en rueda libre: setDuty() y brake() no hacen nada.
  void enable(bool on) {
    enabled_ = on;
    if (!on) coast();
  }
  bool enabled() const { return enabled_; }

  // duty en [−1, 1]. Positivo = avanzar.
  void setDuty(float duty) {
    if (!enabled_) return;
    if (duty > 1) duty = 1;
    if (duty < -1) duty = -1;
    float hw = invert_ ? -duty : duty;
    uint32_t d = (uint32_t)(fabsf(hw) * maxDuty_ * fullScale_ + 0.5f);
    if (d == 0) {
      brake();
      return;
    }
    digitalWrite(p_.in1, hw >= 0 ? HIGH : LOW);
    digitalWrite(p_.in2, hw >= 0 ? LOW : HIGH);
    ledcWrite(p_.ch, d);
    duty_ = duty;
    braking_ = false;
  }

  // Freno activo: IN1 = IN2 = alto con EN al 100 %. Con el canal desactivado queda libre.
  void brake() {
    duty_ = 0;
    if (!enabled_) {
      coast();
      return;
    }
    digitalWrite(p_.in1, HIGH);
    digitalWrite(p_.in2, HIGH);
    ledcWrite(p_.ch, fullScale_);
    braking_ = true;
  }

  // Rueda libre: EN en bajo
  void coast() {
    ledcWrite(p_.ch, 0);
    digitalWrite(p_.in1, LOW);
    digitalWrite(p_.in2, LOW);
    duty_ = 0;
    braking_ = false;
  }

  float duty() const { return duty_; }  // lo que pidió el control, antes del límite maxDuty
  bool braking() const { return braking_; }

 private:
  Pins p_{};
  bool invert_ = false;
  bool enabled_ = false;
  float maxDuty_ = 1.0f;
  uint32_t fullScale_ = 1023;
  float duty_ = 0;
  bool braking_ = false;
};
