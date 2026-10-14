#ifndef SERVO_OUTPUT_H
#define SERVO_OUTPUT_H

#include <Arduino.h>

class ServoOutput {
public:
    void init();
    bool writeAngle(int angle);
    void stop();
    bool active() const { return active_; }

private:
    bool active_ = false;
};

extern ServoOutput servoOutput;

#endif // SERVO_OUTPUT_H
