#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->integral = 0;
  this->prev_error = 0;
  this->max_output = 255.0f; // Giới hạn PWM 8-bit
}

void PIDController::setGains(float kp, float ki, float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

void PIDController::reset() {
  this->integral = 0;
  this->prev_error = 0;
}

float PIDController::compute(float setpoint, float measured, float dt) {
  if (dt <= 0.0f)
    return 0.0f;

  float error = setpoint - measured;

  this->integral += error * dt;

  // Anti-windup (Chống bão hòa khâu tích phân)
  // Nếu ki = 0 thì không cần giới hạn integral, để tránh chia cho 0
  if (this->ki > 0.0f) {
    float max_integral = this->max_output / this->ki;
    if (this->integral > max_integral)
      this->integral = max_integral;
    else if (this->integral < -max_integral)
      this->integral = -max_integral;
  }

  float derivative = (error - this->prev_error) / dt;
  this->prev_error = error;

  float output = (this->kp * error) + (this->ki * this->integral) +
                 (this->kd * derivative);

  // Ràng buộc giới hạn đầu ra (Clamp)
  if (output > this->max_output)
    output = this->max_output;
  else if (output < -this->max_output)
    output = -this->max_output;

  return output;
}
