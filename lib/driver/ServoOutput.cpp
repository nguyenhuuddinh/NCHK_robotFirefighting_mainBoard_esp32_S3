#include "ServoOutput.h"
#include "../common/PinConfig.h"
#include "../common/ServoConfig.h"

ServoOutput servoOutput;

void ServoOutput::init() {
    // No pulse until an explicit calibration command or an enabled live sweep.
    pinMode(SERVO_SIGNAL_PIN, OUTPUT);
    digitalWrite(SERVO_SIGNAL_PIN, LOW);
    active_ = false;
}

bool ServoOutput::writeAngle(int angle) {
    if (angle < SERVO_TEST_MIN_DEG || angle > SERVO_TEST_MAX_DEG) return false;

    if (!active_) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        if (!ledcAttachChannel(SERVO_SIGNAL_PIN, SERVO_PWM_FREQUENCY_HZ,
                               SERVO_PWM_RESOLUTION_BITS, SERVO_PWM_CHANNEL)) return false;
#else
        if (ledcSetup(SERVO_PWM_CHANNEL, SERVO_PWM_FREQUENCY_HZ,
                      SERVO_PWM_RESOLUTION_BITS) == 0) return false;
        ledcAttachPin(SERVO_SIGNAL_PIN, SERVO_PWM_CHANNEL);
#endif
        active_ = true;
    }

    const uint32_t pulse_us = SERVO_PULSE_MIN_US +
        static_cast<uint32_t>(angle) * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / 180U;
    const uint32_t duty = (pulse_us * ((1UL << SERVO_PWM_RESOLUTION_BITS) - 1UL) +
                           (1000000UL / SERVO_PWM_FREQUENCY_HZ) / 2UL) /
                          (1000000UL / SERVO_PWM_FREQUENCY_HZ);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ledcWrite(SERVO_SIGNAL_PIN, duty);
#else
    ledcWrite(SERVO_PWM_CHANNEL, duty);
#endif
    return true;
}

void ServoOutput::stop() {
    if (active_) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        ledcWrite(SERVO_SIGNAL_PIN, 0);
        ledcDetach(SERVO_SIGNAL_PIN);
#else
        ledcWrite(SERVO_PWM_CHANNEL, 0);
        ledcDetachPin(SERVO_SIGNAL_PIN);
#endif
        active_ = false;
    }
    pinMode(SERVO_SIGNAL_PIN, OUTPUT);
    digitalWrite(SERVO_SIGNAL_PIN, LOW);
}
