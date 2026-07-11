#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include "../common/RobotConfig.h"
#include "../common/DataStructs.h"

class Kinematics {
public:
    Kinematics();
    
    // Đầu vào: Vận tốc tịnh tiến v_x (m/s) và vận tốc góc omega_z (rad/s)
    // Đầu ra: Vận tốc cần thiết cho 4 bánh (rad/s)
    MotorSpeeds_t computeWheelSpeeds(float v_x, float omega_z);
    
    // Đầu vào: Vận tốc thực tế đo từ Encoder 4 bánh (rad/s)
    // Đầu ra: Vận tốc tịnh tiến và vận tốc góc thực tế của xe (v_x_out, omega_z_out)
    void computeRobotVelocities(MotorSpeeds_t wheel_speeds, float &v_x_out, float &omega_z_out);
};

extern Kinematics kinematics;

#endif
