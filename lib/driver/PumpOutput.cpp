#include "PumpOutput.h"

PumpOutput pumpOutput;

bool PumpOutput::init() {
    pinMode(PUMP_PWM_PIN, OUTPUT);
    digitalWrite(PUMP_PWM_PIN, LOW);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ready_ = ledcAttach(PUMP_PWM_PIN, 20000, 8);
#else
    ready_ = ledcSetup(kPwmChannel, 20000, 8) > 0;
    if (ready_) ledcAttachPin(PUMP_PWM_PIN, kPwmChannel);
#endif
    if (ready_) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        ledcWrite(PUMP_PWM_PIN, 0);
#else
        ledcWrite(kPwmChannel, 0);
#endif
    }
    return ready_;
}

void PumpOutput::setEnabled(bool enabled) {
    if (!ready_) return;
    if (enabled_ == enabled) return;
    enabled_ = enabled;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ledcWrite(PUMP_PWM_PIN, enabled ? kDuty : 0);
#else
    ledcWrite(kPwmChannel, enabled ? kDuty : 0);
#endif
}
