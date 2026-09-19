#include "Odometry.h"
#include "../common/RobotConfig.h"
#include "OdometryFusion.h"

Odometry odometry;

Odometry::Odometry() { init(); }

void Odometry::init() {
  odom.x = 0;
  odom.y = 0;
  odom.yaw = 0;
  odom.linear_velocity = 0;  // linear_velocity: vận tốc tịnh tiến
  odom.angular_velocity = 0; // angular_velocity: vận tốc góc
  last_alpha = COMP_ALPHA_LOW;
}

void Odometry::update(float v_x, float wz_encoder, float gyro_z, float dt,
                      bool stationary) {
  if (dt <= 0.0f)
    return;

  // ========================================
  // Adaptive Complementary Filter (Sensor Fusion)
  // - Khi xoay (|gyro_z| > threshold): alpha CAO → tin gyro
  //   vi encoder bi truot ngang o co cau skid-steer
  // - Khi di thang: alpha THAP → blend deu gyro + encoder
  // ========================================
  const AngularFusionResult fusion =
      fuseAngularVelocity(wz_encoder, gyro_z, stationary,
                          COMP_GYRO_THRESHOLD, COMP_ALPHA_HIGH,
                          COMP_ALPHA_LOW);
  last_alpha = fusion.alpha;
  const float fused_wz = fusion.angular_velocity;

  // ========================================
  // Tich phan vi tri bang Runge-Kutta bac 2 (Mid-point)
  // Chinh xac hon Euler Forward khi robot dang xoay
  // ========================================
  float half_delta_yaw = fused_wz * dt * 0.5f;
  float mid_yaw = odom.yaw + half_delta_yaw;

  odom.yaw += fused_wz * dt;

  // Xu ly goc bao quanh -PI den PI de tranh goc qua lon
  if (odom.yaw > PI)
    odom.yaw -= 2.0f * PI;
  if (odom.yaw < -PI)
    odom.yaw += 2.0f * PI;

  odom.angular_velocity = fused_wz;

  // Tinh toan thay doi vi tri X, Y (Runge-Kutta bac 2)
  // Su dung goc trung binh (mid_yaw) thay vi goc cuoi (Euler)
  float delta_x = v_x * cosf(mid_yaw) * dt;
  float delta_y = v_x * sinf(mid_yaw) * dt;

  odom.x += delta_x;
  odom.y += delta_y;

  odom.linear_velocity = v_x;
}

OdometryData_t Odometry::getOdometry() { return odom; }

float Odometry::getLastAlpha() { return last_alpha; }
