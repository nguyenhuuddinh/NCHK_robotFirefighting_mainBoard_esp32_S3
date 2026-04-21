#pragma once

class PIDController {
public:
    float compute(float setpoint, float current);
};
