#ifndef MOTION_STATIONARY_GATE_H
#define MOTION_STATIONARY_GATE_H

#include <math.h>
#include <stdint.h>

// Chi bao robot dung sau khi dong co da duoc yeu cau dung va encoder im lang
// lien tuc. Khoang xac nhan ngan giup loai gyro bias ma khong che mat chuyen
// dong that khi robot dang chay hoac bi day bang tay.
class MotionStationaryGate {
public:
  MotionStationaryGate(uint16_t confirm_cycles, float cmd_epsilon,
                       float ramp_epsilon)
      : confirm_cycles_(confirm_cycles), cmd_epsilon_(cmd_epsilon),
        ramp_epsilon_(ramp_epsilon), stable_cycles_(0) {}

  bool update(bool motion_allowed, float cmd_vx, float cmd_wz, float ramp_vx,
              float ramp_wz, int64_t delta_fl, int64_t delta_rl,
              int64_t delta_fr, int64_t delta_rr) {
    const bool stop_requested =
        !motion_allowed ||
        (fabsf(cmd_vx) <= cmd_epsilon_ && fabsf(cmd_wz) <= cmd_epsilon_ &&
         fabsf(ramp_vx) <= ramp_epsilon_ &&
         fabsf(ramp_wz) <= ramp_epsilon_);
    const bool encoders_still = delta_fl == 0 && delta_rl == 0 &&
                                delta_fr == 0 && delta_rr == 0;

    if (!stop_requested || !encoders_still) {
      stable_cycles_ = 0;
      return false;
    }

    if (stable_cycles_ < confirm_cycles_) {
      ++stable_cycles_;
    }
    return stable_cycles_ >= confirm_cycles_;
  }

  void reset() { stable_cycles_ = 0; }

private:
  uint16_t confirm_cycles_;
  float cmd_epsilon_;
  float ramp_epsilon_;
  uint16_t stable_cycles_;
};

#endif // MOTION_STATIONARY_GATE_H
