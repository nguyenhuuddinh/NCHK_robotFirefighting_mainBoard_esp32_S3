#include "Kinematics.h"

Kinematics kinematics;

Kinematics::Kinematics() {}

MotorSpeeds_t Kinematics::computeWheelSpeeds(float v_x, float omega_z) {
    MotorSpeeds_t speeds;
    
    // Động học Skid-Steer (4 bánh lái trượt):
    // 2 bánh bên trái quay cùng tốc độ, 2 bánh bên phải quay cùng tốc độ.
    // V_left  = V_x - (omega_z * TRACK_WIDTH / 2)
    // V_right = V_x + (omega_z * TRACK_WIDTH / 2)
    
    float v_left = v_x - (omega_z * TRACK_WIDTH_M / 2.0f);
    float v_right = v_x + (omega_z * TRACK_WIDTH_M / 2.0f);
    
    // Chuyển đổi từ m/s sang rad/s cho bánh xe: omega_wheel = v_wheel / r
    speeds.v_fl = v_left / WHEEL_RADIUS_M;
    speeds.v_rl = v_left / WHEEL_RADIUS_M;
    
    speeds.v_fr = v_right / WHEEL_RADIUS_M;
    speeds.v_rr = v_right / WHEEL_RADIUS_M;
    
    return speeds;
}

void Kinematics::computeRobotVelocities(MotorSpeeds_t wheel_speeds, float &v_x_out, float &omega_z_out) {
    // Chuyển từ rad/s (bánh xe) sang m/s (bánh xe)
    float v_fl_m = wheel_speeds.v_fl * WHEEL_RADIUS_M;
    float v_rl_m = wheel_speeds.v_rl * WHEEL_RADIUS_M;
    float v_fr_m = wheel_speeds.v_fr * WHEEL_RADIUS_M;
    float v_rr_m = wheel_speeds.v_rr * WHEEL_RADIUS_M;
    
    // Vận tốc trung bình bên trái và bên phải
    float v_left_avg = (v_fl_m + v_rl_m) / 2.0f;
    float v_right_avg = (v_fr_m + v_rr_m) / 2.0f;
    
    // Vận tốc tổng của xe
    v_x_out = (v_left_avg + v_right_avg) / 2.0f;
    omega_z_out = (v_right_avg - v_left_avg) / TRACK_WIDTH_M;
}
