#pragma once

#include "PIDController.h"
#include "RobotConfig.h"

/**
 * @brief Cac ham tinh toan dieu khien motor.
 *
 * Tap trung logic Feed-Forward + PID + Ramp vao 1 noi.
 * Truoc do nam rai rac trong main.cpp.
 */
namespace MotionControl {

/**
 * @brief Tinh Feed-Forward PWM de bo ma sat tinh.
 * @param omega_tgt  Van toc muc tieu (rad/s)
 * @param offset_pwm PWM nguong vuot ma sat (FF_PWM_OFFSET hoac FF_PWM_OFFSET_TURN)
 * @return PWM Feed-Forward (co dau, chua clamp)
 */
inline float feedForward(float omega_tgt, float offset_pwm) {
    if (omega_tgt > 0.01f)
        return offset_pwm + omega_tgt * FF_K_FF_ACTIVE;
    if (omega_tgt < -0.01f)
        return -offset_pwm + omega_tgt * FF_K_FF_ACTIVE;
    return 0.0f;
}

/**
 * @brief Tinh tong PWM = Feed-Forward + PID, roi clamp [-PWM_MAX, PWM_MAX].
 * @param omega_tgt  Van toc muc tieu (rad/s)
 * @param omega_act  Van toc thuc te (rad/s)
 * @param pid        Bo dieu khien PID cua banh tuong ung
 * @param offset_pwm PWM offset (chon theo di thang / xoay)
 * @param dt         Chu ky tinh toan (s)
 * @return PWM cuoi cung da clamp
 */
inline float computePWM(float omega_tgt, float omega_act,
                         PIDController& pid, float offset_pwm, float dt) {
    float ff = feedForward(omega_tgt, offset_pwm);
    float pid_out = pid.compute(omega_tgt, omega_act, dt);
    float total = ff + pid_out;
    // Clamp trong gioi han PWM
    if (total > PWM_MAX) total = PWM_MAX;
    if (total < -PWM_MAX) total = -PWM_MAX;
    return total;
}

/**
 * @brief Chon offset PWM phu hop theo kieu di chuyen.
 * Xoay thuan tuy (wz lon, vx nho) dung offset cao hon.
 * @param cmd_vx Van toc tuyen tinh hien tai
 * @param cmd_wz Van toc goc hien tai
 * @return PWM offset tuong ung
 */
inline float selectOffset(float cmd_vx, float cmd_wz) {
    if (fabsf(cmd_wz) > 0.01f && fabsf(cmd_vx) < 0.01f)
        return FF_PWM_OFFSET_TURN; // Xoay thuan tuy
    return FF_PWM_OFFSET;          // Di thang hoac ket hop
}

/**
 * @brief Ap dung Ramp (tang/giam toc mem) de bao ve banh rang.
 * @param current Gia tri hien tai (dang ramp)
 * @param target  Gia tri muc tieu
 * @param max_rate Toc do tang/giam toi da (don vi/s)
 * @param dt      Chu ky tinh toan (s)
 * @return Gia tri moi sau ramp
 */
inline float applyRamp(float current, float target, float max_rate, float dt) {
    float delta = target - current;
    float max_step = max_rate * dt;
    if (fabsf(delta) <= max_step)
        return target;
    return current + copysignf(max_step, delta);
}

} // namespace MotionControl
