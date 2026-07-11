#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    
    void setGains(float kp, float ki, float kd);
    void reset();
    
    // Tính toán giá trị đầu ra (PWM) dựa trên Setpoint và giá trị thực tế
    // dt là khoảng thời gian lấy mẫu (giây)
    float compute(float setpoint, float measured, float dt);

private:
    float kp, ki, kd;
    float integral;
    float prev_error;
    float max_output; // Giới hạn xung ra (thường là 255)
};

#endif
