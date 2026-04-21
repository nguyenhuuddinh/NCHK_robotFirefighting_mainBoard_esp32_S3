#pragma once

class ServoDriver {
public:
    void init();
    void setAngle(int channel, int angle);
};
