#ifndef ODOMETRY_FUSION_H
#define ODOMETRY_FUSION_H

#include <math.h>

struct AngularFusionResult {
  float angular_velocity;
  float alpha;
};

inline AngularFusionResult
fuseAngularVelocity(float wz_encoder, float gyro_z, bool stationary,
                    float gyro_threshold, float alpha_high, float alpha_low) {
  if (stationary) {
    // Khong tich phan gyro bias khi robot da dung co chu dich va encoder im.
    return {0.0f, 0.0f};
  }

  const float alpha =
      fabsf(gyro_z) > gyro_threshold ? alpha_high : alpha_low;
  return {alpha * gyro_z + (1.0f - alpha) * wz_encoder, alpha};
}

#endif // ODOMETRY_FUSION_H
