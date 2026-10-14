#ifndef SERVO_SWEEP_H
#define SERVO_SWEEP_H

#include <stdint.h>

// Pure, allocation-free two-leg sweep with a gated return to the start angle.
// No position feedback:
// the operator must physically park the nozzle at start_deg before first ON.
class ServoSweep {
public:
    ServoSweep(int start_deg, int turn_deg, int end_deg, uint32_t step_period_ms)
        : start_deg_(start_deg), turn_deg_(turn_deg), end_deg_(end_deg),
          step_period_ms_(step_period_ms) {}

    // Returns -1 when servo must be detached; otherwise a bounded angle.
    int update(bool pump_software_on, uint32_t now_ms) {
        if (homing_) {
            if (angle_ != start_deg_ &&
                static_cast<uint32_t>(now_ms - last_step_ms_) >= step_period_ms_) {
                angle_ += start_deg_ > angle_ ? 1 : -1;
                last_step_ms_ = now_ms;
                if (angle_ == start_deg_) {
                    home_at_ms_ = now_ms;
                    home_reached_ = true;
                }
            }
            return angle_;
        }
        if (!pump_software_on) {
            if (active_) consumed_ = true;
            active_ = false;
            return -1;
        }
        if (consumed_ || step_period_ms_ == 0 || !reversesDirection()) return -1;
        if (!active_) {
            active_ = true;
            returning_ = false;
            angle_ = start_deg_;
            last_step_ms_ = now_ms;
            return angle_;
        }
        const int target = returning_ ? end_deg_ : turn_deg_;
        if (angle_ != target &&
            static_cast<uint32_t>(now_ms - last_step_ms_) >= step_period_ms_) {
            angle_ += target > angle_ ? 1 : -1;
            last_step_ms_ = now_ms; // Never catch up with a multi-degree jump.
            if (!returning_ && angle_ == turn_deg_) returning_ = true;
        }
        return angle_;
    }

    // Caller must first switch off pump and verify an intentional stop (or
    // the 5 s run cap), a fresh zero drive command and a live host/watchdog.
    bool beginReturn(uint32_t now_ms) {
        if (!active_ || angle_ != end_deg_ || homing_) return false;
        active_ = false;
        homing_ = true;
        consumed_ = true;
        last_step_ms_ = now_ms;
        home_reached_ = false;
        return true;
    }
    // Keep PWM at 90 degrees long enough to settle before detaching.
    bool returnSettled(uint32_t now_ms) const {
        return homing_ && home_reached_ &&
               static_cast<uint32_t>(now_ms - home_at_ms_) >= 300;
    }
    bool rearm(uint32_t now_ms) {
        if (!returnSettled(now_ms)) return false;
        homing_ = false;
        consumed_ = false;
        return true;
    }
    void abort() { active_ = false; homing_ = false; consumed_ = true; }
    bool consumed() const { return consumed_; }
    bool active() const { return active_; }
    bool returningHome() const { return homing_; }
    bool atEnd() const { return active_ && angle_ == end_deg_; }

private:
    bool reversesDirection() const {
        return (turn_deg_ < start_deg_ && end_deg_ > turn_deg_) ||
               (turn_deg_ > start_deg_ && end_deg_ < turn_deg_);
    }

    int start_deg_;
    int turn_deg_;
    int end_deg_;
    uint32_t step_period_ms_;
    uint32_t last_step_ms_ = 0;
    int angle_ = 0;
    bool active_ = false;
    bool returning_ = false;
    bool homing_ = false;
    bool consumed_ = false;
    bool home_reached_ = false;
    uint32_t home_at_ms_ = 0;
};

#endif // SERVO_SWEEP_H
