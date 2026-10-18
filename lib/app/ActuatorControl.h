#pragma once

#include <stdint.h>

// Only Task_Actuator writes pump/servo PWM. Serial submits validated requests
// and the current host state; neither API waits for the actuator task.
struct ActuatorStatus {
    bool pumpOn = false;
    bool pumpReady = false;
    bool servoActive = false;
    uint8_t phase = 0;
    uint8_t angle = 255;
    uint32_t homeCompletions = 0;
};

void actuatorSubmitPump(bool on, uint32_t received_ms);
void actuatorPublishLink(bool online, uint32_t tick_ms);
ActuatorStatus actuatorStatusSnapshot();
