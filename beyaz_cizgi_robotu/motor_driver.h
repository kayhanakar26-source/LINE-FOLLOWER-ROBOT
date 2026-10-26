#pragma once

#include <Arduino.h>

#include "config.h"

// ============================================================
//  Tek motor kanalı
//  4 motorlu robotta her kanal 2 motoru paralel sürer.
//  speed: -MAX_PWM..+MAX_PWM (pozitif = ileri)
// ============================================================
class MotorChannel {
 public:
  constexpr MotorChannel(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, bool inverted)
      : pwmPin_(pwmPin), in1Pin_(in1Pin), in2Pin_(in2Pin), inverted_(inverted) {}

  void begin() const {
    pinMode(pwmPin_, OUTPUT);
    pinMode(in1Pin_, OUTPUT);
    pinMode(in2Pin_, OUTPUT);
    stop();
  }

  void setSpeed(int speed) const {
    speed = constrain(speed, -MAX_PWM, MAX_PWM);

    if (inverted_) {
      speed = -speed;
    }

    if (speed > 0) {
      digitalWrite(in1Pin_, HIGH);
      digitalWrite(in2Pin_, LOW);
    } else if (speed < 0) {
      digitalWrite(in1Pin_, LOW);
      digitalWrite(in2Pin_, HIGH);
    } else {
      digitalWrite(in1Pin_, LOW);
      digitalWrite(in2Pin_, LOW);
    }

    analogWrite(pwmPin_, abs(speed));
  }

  void stop() const {
    digitalWrite(in1Pin_, LOW);
    digitalWrite(in2Pin_, LOW);
    analogWrite(pwmPin_, 0);
  }

 private:
  uint8_t pwmPin_;
  uint8_t in1Pin_;
  uint8_t in2Pin_;
  bool inverted_;
};
