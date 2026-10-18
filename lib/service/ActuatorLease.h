#pragma once

#include <stdint.h>

// Faster than the 750 ms PUMP lease: an actuator owner must notice that the
// serial task itself stopped, even when no new PUMP command can arrive.
static constexpr uint32_t kActuatorSerialLeaseMs = 250;

static inline bool actuatorSerialLinkFresh(bool online, uint32_t tick_ms,
                                           uint32_t now_ms) {
    return online && (uint32_t)(now_ms - tick_ms) <= kActuatorSerialLeaseMs;
}

// An OFF followed by ON before the owner ticks never drove PWM low. Do not
// let that fleeting OFF re-arm a continuous run or bypass the 5 s cap.
static inline bool actuatorObservedOff(uint32_t off_revision,
                                       uint32_t seen_off_revision,
                                       bool latest_on) {
    return off_revision != seen_off_revision && !latest_on;
}
