#ifndef PUMP_OUTPUT_H
#define PUMP_OUTPUT_H

#include <Arduino.h>
#include "../common/PinConfig.h"

class PumpOutput {
public:
    bool init();
    void setEnabled(bool enabled);
    bool ready() const { return ready_; }
    bool commandedOn() const { return ready_ && enabled_; }

private:
    bool ready_ = false;
    bool enabled_ = false;
    static constexpr uint8_t kPwmChannel = 4; // Motor PWM uses channels 0-3.
    static constexpr uint8_t kDuty = 255;      // Fixed demo duty; tune later.
};

extern PumpOutput pumpOutput;

#endif // PUMP_OUTPUT_H
