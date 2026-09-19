#include <unity.h>

#include "../../lib/service/MotionStationaryGate.h"
#include "../../lib/service/OdometryFusion.h"

void setUp(void) {}
void tearDown(void) {}

void test_requires_consecutive_stationary_cycles(void) {
  MotionStationaryGate gate(3, 0.0001f, 0.01f);

  TEST_ASSERT_FALSE(gate.update(true, 0.0f, 0.0f, 0.0f, 0.0f,
                                0, 0, 0, 0));
  TEST_ASSERT_FALSE(gate.update(true, 0.0f, 0.0f, 0.0f, 0.0f,
                                0, 0, 0, 0));
  TEST_ASSERT_TRUE(gate.update(true, 0.0f, 0.0f, 0.0f, 0.0f,
                               0, 0, 0, 0));
}

void test_nonzero_command_or_ramp_prevents_stationary(void) {
  MotionStationaryGate gate(2, 0.0001f, 0.01f);

  TEST_ASSERT_FALSE(gate.update(true, 0.05f, 0.0f, 0.0f, 0.0f,
                                0, 0, 0, 0));
  TEST_ASSERT_FALSE(gate.update(true, 0.0f, 0.0f, 0.02f, 0.0f,
                                0, 0, 0, 0));
  TEST_ASSERT_FALSE(gate.update(true, 0.0f, 0.0f, 0.0f, 0.0f,
                                0, 0, 0, 0));
  TEST_ASSERT_TRUE(gate.update(true, 0.0f, 0.0f, 0.0f, 0.0f,
                               0, 0, 0, 0));
}

void test_encoder_tick_resets_stationary_confirmation(void) {
  MotionStationaryGate gate(2, 0.0001f, 0.01f);

  TEST_ASSERT_FALSE(gate.update(true, 0.0f, 0.0f, 0.0f, 0.0f,
                                0, 0, 0, 0));
  TEST_ASSERT_FALSE(gate.update(true, 0.0f, 0.0f, 0.0f, 0.0f,
                                0, 1, 0, 0));
  TEST_ASSERT_FALSE(gate.update(true, 0.0f, 0.0f, 0.0f, 0.0f,
                                0, 0, 0, 0));
  TEST_ASSERT_TRUE(gate.update(true, 0.0f, 0.0f, 0.0f, 0.0f,
                               0, 0, 0, 0));
}

void test_disabled_motion_still_requires_encoder_silence(void) {
  MotionStationaryGate gate(2, 0.0001f, 0.01f);

  TEST_ASSERT_FALSE(gate.update(false, 1.0f, 1.0f, 1.0f, 1.0f,
                                0, 0, 0, 0));
  TEST_ASSERT_FALSE(gate.update(false, 1.0f, 1.0f, 1.0f, 1.0f,
                                1, 0, 0, 0));
  TEST_ASSERT_FALSE(gate.update(false, 1.0f, 1.0f, 1.0f, 1.0f,
                                0, 0, 0, 0));
  TEST_ASSERT_TRUE(gate.update(false, 1.0f, 1.0f, 1.0f, 1.0f,
                               0, 0, 0, 0));
}

void test_stationary_fusion_rejects_gyro_bias(void) {
  const AngularFusionResult result =
      fuseAngularVelocity(0.0f, 0.00236f, true, 0.1f, 0.995f, 0.95f);

  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f, result.angular_velocity);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f, result.alpha);
}

void test_moving_fusion_keeps_existing_filter_behavior(void) {
  const AngularFusionResult low =
      fuseAngularVelocity(0.02f, 0.04f, false, 0.1f, 0.995f, 0.95f);
  const AngularFusionResult high =
      fuseAngularVelocity(0.20f, 0.30f, false, 0.1f, 0.995f, 0.95f);

  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.95f, low.alpha);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.039f, low.angular_velocity);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.995f, high.alpha);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.2995f, high.angular_velocity);
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_requires_consecutive_stationary_cycles);
  RUN_TEST(test_nonzero_command_or_ramp_prevents_stationary);
  RUN_TEST(test_encoder_tick_resets_stationary_confirmation);
  RUN_TEST(test_disabled_motion_still_requires_encoder_silence);
  RUN_TEST(test_stationary_fusion_rejects_gyro_bias);
  RUN_TEST(test_moving_fusion_keeps_existing_filter_behavior);
  return UNITY_END();
}
