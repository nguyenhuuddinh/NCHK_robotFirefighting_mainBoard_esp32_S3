#include <unity.h>
#include "../../lib/common/ServoConfig.h"
#include "../../lib/service/PumpSafety.h"
#include "../../lib/service/ServoSweep.h"

void setUp(void) {}
void tearDown(void) {}

void test_configured_first_angle_is_ninety(void) {
    TEST_ASSERT_EQUAL_INT(90, SERVO_SWEEP_START_DEG);
    TEST_ASSERT_EQUAL_INT(45, SERVO_SWEEP_TURN_DEG);
    TEST_ASSERT_EQUAL_INT(115, SERVO_SWEEP_END_DEG);
    TEST_ASSERT_EQUAL_UINT32(35, SERVO_SWEEP_STEP_PERIOD_MS);
    ServoSweep sweep(SERVO_SWEEP_START_DEG, SERVO_SWEEP_TURN_DEG, SERVO_SWEEP_END_DEG,
                     SERVO_SWEEP_STEP_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(90, sweep.update(true, 0));
}

void test_off_consumes_one_shot_and_prevents_second_start_jump(void) {
    ServoSweep sweep(90, 45, 115, 35);
    TEST_ASSERT_EQUAL_INT(-1, sweep.update(false, 100));
    TEST_ASSERT_EQUAL_INT(90, sweep.update(true, 100));
    TEST_ASSERT_EQUAL_INT(89, sweep.update(true, 140));
    TEST_ASSERT_EQUAL_INT(-1, sweep.update(false, 170));
    TEST_ASSERT_TRUE(sweep.consumed());
    TEST_ASSERT_EQUAL_INT(-1, sweep.update(true, 200));
}

void test_never_jumps_more_than_one_degree_even_after_stall(void) {
    ServoSweep sweep(90, 45, 115, 35);
    TEST_ASSERT_EQUAL_INT(90, sweep.update(true, 0));
    TEST_ASSERT_EQUAL_INT(90, sweep.update(true, 34));
    TEST_ASSERT_EQUAL_INT(89, sweep.update(true, 35));
    TEST_ASSERT_EQUAL_INT(88, sweep.update(true, 10000));
    TEST_ASSERT_EQUAL_INT(88, sweep.update(true, 10020));
    TEST_ASSERT_EQUAL_INT(87, sweep.update(true, 10040));
}

void test_full_path_fits_five_seconds_at_nominal_tick(void) {
    ServoSweep sweep(90, 45, 115, 35);
    int previous = sweep.update(true, 0);
    int turn_at = -1;
    int end_at = -1;
    for (int t = 20; t <= 5000; t += 20) {
        int angle = sweep.update(true, static_cast<uint32_t>(t));
        TEST_ASSERT_TRUE(angle >= 45 && angle <= 115);
        TEST_ASSERT_TRUE(angle >= previous - 1 && angle <= previous + 1);
        if (angle == 45 && turn_at < 0) turn_at = t;
        if (angle == 115 && end_at < 0) end_at = t;
        previous = angle;
    }
    TEST_ASSERT_EQUAL_INT(1800, turn_at);
    TEST_ASSERT_EQUAL_INT(4600, end_at);
    TEST_ASSERT_EQUAL_INT(115, previous);
    TEST_ASSERT_FALSE(sweep.consumed());
}

void test_turning_point_does_not_jump(void) {
    ServoSweep sweep(90, 45, 115, 35);
    sweep.update(true, 0);
    for (int t = 20; t <= 1800; t += 20) sweep.update(true, t);
    TEST_ASSERT_EQUAL_INT(45, sweep.update(true, 1820));
    TEST_ASSERT_EQUAL_INT(46, sweep.update(true, 1840));
}

void test_reversed_mount_can_step_up_in_degrees(void) {
    ServoSweep sweep(90, 115, 45, 35);
    TEST_ASSERT_EQUAL_INT(90, sweep.update(true, 100));
    TEST_ASSERT_EQUAL_INT(91, sweep.update(true, 140));
    TEST_ASSERT_EQUAL_INT(92, sweep.update(true, 180));
}

void test_clock_wrap_is_handled(void) {
    ServoSweep sweep(90, 45, 115, 35);
    TEST_ASSERT_EQUAL_INT(90, sweep.update(true, UINT32_MAX - 19));
    TEST_ASSERT_EQUAL_INT(89, sweep.update(true, 20));
}

void test_abort_and_invalid_config_never_emit_more_angles(void) {
    ServoSweep sweep(90, 45, 115, 35);
    TEST_ASSERT_EQUAL_INT(90, sweep.update(true, 0));
    sweep.abort();
    TEST_ASSERT_EQUAL_INT(-1, sweep.update(true, 40));
    ServoSweep no_range(90, 90, 115, 35);
    TEST_ASSERT_EQUAL_INT(-1, no_range.update(true, 0));
    ServoSweep no_reverse(90, 45, 30, 35);
    TEST_ASSERT_EQUAL_INT(-1, no_reverse.update(true, 0));
    ServoSweep no_period(90, 45, 115, 0);
    TEST_ASSERT_EQUAL_INT(-1, no_period.update(true, 0));
}

void test_pump_five_second_cap_detaches_and_latches_sweep(void) {
    PumpSafety pump;
    ServoSweep sweep(90, 45, 115, 35);
    pump.request(true, 0);
    TEST_ASSERT_TRUE(pump.tick(0, true));
    TEST_ASSERT_EQUAL_INT(90, sweep.update(true, 0));
    for (uint32_t t = 20; t < 5000; t += 20) {
        if (t % 100 == 0) pump.request(true, t);
        TEST_ASSERT_TRUE(pump.tick(t, true));
        sweep.update(true, t);
    }
    pump.request(true, 5000);
    TEST_ASSERT_FALSE(pump.tick(5000, true));
    TEST_ASSERT_EQUAL_INT(PumpSafety::StopReason::MAX_RUN, pump.stopReason());
    TEST_ASSERT_TRUE(sweep.atEnd());
    TEST_ASSERT_TRUE(sweep.beginReturn(5000));
    TEST_ASSERT_EQUAL_INT(115, sweep.update(false, 5000));
    TEST_ASSERT_TRUE(sweep.consumed());
    pump.request(false, 5020);
    pump.request(true, 5040);
    TEST_ASSERT_TRUE(pump.tick(5040, true));
    TEST_ASSERT_EQUAL_INT(114, sweep.update(true, 5040));
    TEST_ASSERT_TRUE(sweep.consumed());
    int previous = 114;
    for (uint32_t t = 5080; t <= 6000; t += 40) {
        int angle = sweep.update(false, t);
        TEST_ASSERT_TRUE(angle >= previous - 1 && angle <= previous);
        previous = angle;
    }
    TEST_ASSERT_EQUAL_INT(90, previous);
    TEST_ASSERT_FALSE(sweep.returnSettled(6000));
    TEST_ASSERT_TRUE(sweep.returnSettled(6300));
    // A new ON without an intervening OFF must not rearm; the caller only
    // calls rearm when the wire-level PUMP request has returned to zero.
    TEST_ASSERT_EQUAL_INT(90, sweep.update(true, 6300));
    TEST_ASSERT_TRUE(sweep.consumed());
    pump.request(false, 6320);
    TEST_ASSERT_TRUE(sweep.rearm(6320));
    TEST_ASSERT_FALSE(sweep.consumed());
    TEST_ASSERT_EQUAL_INT(-1, sweep.update(false, 6320));
    pump.request(true, 6360);
    TEST_ASSERT_TRUE(pump.tick(6360, true));
    TEST_ASSERT_EQUAL_INT(90, sweep.update(true, 6360));
}

void test_early_stop_cannot_home_or_rearm(void) {
    ServoSweep sweep(90, 45, 115, 35);
    TEST_ASSERT_EQUAL_INT(90, sweep.update(true, 0));
    TEST_ASSERT_EQUAL_INT(89, sweep.update(true, 40));
    TEST_ASSERT_FALSE(sweep.beginReturn(50));
    TEST_ASSERT_EQUAL_INT(-1, sweep.update(false, 50));
    TEST_ASSERT_TRUE(sweep.consumed());
    TEST_ASSERT_FALSE(sweep.rearm(10000));
    TEST_ASSERT_EQUAL_INT(-1, sweep.update(true, 10020));
}

void test_home_interruption_remains_latched(void) {
    ServoSweep sweep(90, 45, 115, 35);
    sweep.update(true, 0);
    for (uint32_t t = 40; t <= 4600; t += 40) sweep.update(true, t);
    TEST_ASSERT_TRUE(sweep.beginReturn(5000));
    TEST_ASSERT_EQUAL_INT(114, sweep.update(false, 5040));
    sweep.abort(); // host/watchdog/nonzero drive command
    TEST_ASSERT_EQUAL_INT(-1, sweep.update(false, 5080));
    TEST_ASSERT_FALSE(sweep.rearm(10000));
}

void test_explicit_off_after_full_sweep_can_home(void) {
    PumpSafety pump;
    ServoSweep sweep(90, 45, 115, 35);
    pump.request(true, 0);
    sweep.update(pump.tick(0, true), 0);
    for (uint32_t t = 20; t <= 4600; t += 20) {
        if (t % 100 == 0) pump.request(true, t);
        sweep.update(pump.tick(t, true), t);
    }
    TEST_ASSERT_TRUE(sweep.atEnd());
    pump.request(false, 4620);
    TEST_ASSERT_FALSE(pump.tick(4620, true));
    TEST_ASSERT_EQUAL_INT(PumpSafety::StopReason::NONE, pump.stopReason());
    TEST_ASSERT_TRUE(sweep.beginReturn(4620));
    TEST_ASSERT_EQUAL_INT(115, sweep.update(false, 4620));
}

void test_pump_stop_reason_distinguishes_cap_from_faults(void) {
    PumpSafety pump;
    pump.request(true, 0);
    TEST_ASSERT_FALSE(pump.tick(20, false));
    TEST_ASSERT_EQUAL_INT(PumpSafety::StopReason::HOST_LOST, pump.stopReason());
    pump.request(false, 40);
    pump.request(true, 60);
    TEST_ASSERT_FALSE(pump.tick(820, true));
    TEST_ASSERT_EQUAL_INT(PumpSafety::StopReason::LEASE_EXPIRED, pump.stopReason());
    pump.request(false, 840);
    TEST_ASSERT_EQUAL_INT(PumpSafety::StopReason::NONE, pump.stopReason());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_configured_first_angle_is_ninety);
    RUN_TEST(test_off_consumes_one_shot_and_prevents_second_start_jump);
    RUN_TEST(test_never_jumps_more_than_one_degree_even_after_stall);
    RUN_TEST(test_full_path_fits_five_seconds_at_nominal_tick);
    RUN_TEST(test_turning_point_does_not_jump);
    RUN_TEST(test_reversed_mount_can_step_up_in_degrees);
    RUN_TEST(test_clock_wrap_is_handled);
    RUN_TEST(test_abort_and_invalid_config_never_emit_more_angles);
    RUN_TEST(test_pump_five_second_cap_detaches_and_latches_sweep);
    RUN_TEST(test_early_stop_cannot_home_or_rearm);
    RUN_TEST(test_home_interruption_remains_latched);
    RUN_TEST(test_explicit_off_after_full_sweep_can_home);
    RUN_TEST(test_pump_stop_reason_distinguishes_cap_from_faults);
    return UNITY_END();
}
