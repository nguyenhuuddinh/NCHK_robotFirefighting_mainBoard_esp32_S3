#ifndef PUMP_SAFETY_H
#define PUMP_SAFETY_H

#include <stdint.h>

// Core-0 only. A fresh PUMP=0 is required after any timeout or disconnect.
class PumpSafety {
public:
    enum class StopReason { NONE, HOST_LOST, LEASE_EXPIRED, MAX_RUN };
    static constexpr uint32_t kLeaseMs = 750;
    static constexpr uint32_t kMaxRunMs = 5000;

    void request(bool on, uint32_t now) {
        if (!on) {
            requested_ = false;
            locked_off_ = false;
            started_ = false;
            stop_reason_ = StopReason::NONE;
            return;
        }
        if (locked_off_) return;
        if (!started_) {
            started_at_ = now;
            started_ = true;
        }
        requested_ = true;
        last_on_at_ = now;
    }

    bool tick(uint32_t now, bool host_online) {
        if (requested_ && !locked_off_) {
            if (!host_online) stop_reason_ = StopReason::HOST_LOST;
            else if ((uint32_t)(now - last_on_at_) > kLeaseMs)
                stop_reason_ = StopReason::LEASE_EXPIRED;
            else if ((uint32_t)(now - started_at_) >= kMaxRunMs)
                stop_reason_ = StopReason::MAX_RUN;
            if (stop_reason_ != StopReason::NONE) locked_off_ = true;
        }
        return requested_ && !locked_off_ && host_online;
    }

    bool lockedOff() const { return locked_off_; }
    StopReason stopReason() const { return stop_reason_; }

private:
    bool requested_ = false;
    bool locked_off_ = false;
    StopReason stop_reason_ = StopReason::NONE;
    bool started_ = false;
    uint32_t started_at_ = 0;
    uint32_t last_on_at_ = 0;
};

#endif // PUMP_SAFETY_H
