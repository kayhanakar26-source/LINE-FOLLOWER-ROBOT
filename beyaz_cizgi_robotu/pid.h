#pragma once

#include <Arduino.h>

// ============================================================
//  PID denetleyici
//  Hata birimi: -1..+1 (çizgi konumu)
//  Çıkış birimi: PWM (-255..+255)
//
//  - I terimi sınırlandırılır (integral windup koruması).
//  - Türev, alçak geçiren filtre ile yumuşatılır.
//  - Türev ilk çevrimde devre dışı bırakılır (ilk sıçramayı önler).
// ============================================================
class PidController {
 public:
  PidController(float kp, float ki, float kd, float integralLimit, float derivativeFilter)
      : kp_(kp),
        ki_(ki),
        kd_(kd),
        integralLimit_(integralLimit),
        derivativeFilter_(constrain(derivativeFilter, 0.0f, 0.99f)) {}

  void reset() {
    integral_ = 0.0f;
    derivative_ = 0.0f;
    previousError_ = 0.0f;
    firstUpdate_ = true;
  }

  float update(float error, float dtSeconds) {
    if (dtSeconds <= 0.0f) {
      return 0.0f;
    }

    // Döngü gecikirse türev patlamasın.
    if (dtSeconds > 0.1f) {
      dtSeconds = 0.1f;
    }

    const float proportional = kp_ * error;

    integral_ += ki_ * error * dtSeconds;
    integral_ = constrain(integral_, -integralLimit_, integralLimit_);

    float derivative = 0.0f;

    if (!firstUpdate_) {
      const float rawDerivative = (error - previousError_) / dtSeconds;

      // derivative_ += (raw - derivative_) * (1 - filtre)
      derivative_ += (rawDerivative - derivative_) * (1.0f - derivativeFilter_);
      derivative = kd_ * derivative_;
    }

    previousError_ = error;
    firstUpdate_ = false;

    return proportional + integral_ + derivative;
  }

 private:
  float kp_;
  float ki_;
  float kd_;
  float integralLimit_;
  float derivativeFilter_;

  float integral_ = 0.0f;
  float derivative_ = 0.0f;
  float previousError_ = 0.0f;
  bool firstUpdate_ = true;
};
