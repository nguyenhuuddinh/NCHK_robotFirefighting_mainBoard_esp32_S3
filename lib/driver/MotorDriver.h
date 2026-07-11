#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include "../common/PinConfig.h"

class MotorDriver {
public:
    MotorDriver();
    void init();
    
    // Vận tốc (speed) từ -255 đến 255
    void setMotorFL(int speed); // Front Left  (Motor 1)
    void setMotorRL(int speed); // Rear Left   (Motor 2)
    void setMotorFR(int speed); // Front Right (Motor 3)
    void setMotorRR(int speed); // Rear Right  (Motor 4)
    
    void setSpeeds(int fl, int rl, int fr, int rr);
    void stopAll();

private:
    const int pwmFreq = 20000;   // Tần số 20kHz cho L298N tránh tiếng ồn
    const int pwmResolution = 8; // Độ phân giải 8-bit (0-255)

    // ESP32 PWM Channels (dành cho API cũ)
    const int chFL = 0;
    const int chRL = 1;
    const int chFR = 2;
    const int chRR = 3;
};

extern MotorDriver motorDriver;

#endif
