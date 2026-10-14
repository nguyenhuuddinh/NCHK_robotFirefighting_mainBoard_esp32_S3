#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

// GPIO38 SG90 nozzle; calibration has a separate firmware environment.
#ifndef SERVO_CALIBRATION_MODE
#define SERVO_CALIBRATION_MODE 0
#endif

// PUMP=1 in normal firmware also drives GPIO38 through the guarded sweep.
// The separate calibration firmware must never run the live pump path.
#if SERVO_CALIBRATION_MODE
#define SERVO_SWEEP_ENABLED 0
#else
#define SERVO_SWEEP_ENABLED 1
#endif
#define SERVO_LIMITS_CONFIRMED 1
#define SERVO_TRAJECTORY_CONFIRMED 1

// Manual calibration only; widen further only after observing clearances.
#define SERVO_TEST_MIN_DEG 40
#define SERVO_TEST_MAX_DEG 140
#define SERVO_CAL_HOLD_MAX_MS 10000UL

// Physically park the nozzle near 90 before the first live pump sweep.
// One degree per >=35 ms; with the 20 ms Serial task, 115 steps take ~4.6 s.
// PumpSafety's 5 s hard cap remains unchanged. No catch-up on a delayed task.
#define SERVO_SWEEP_START_DEG 90
#define SERVO_SWEEP_TURN_DEG 45
#define SERVO_SWEEP_END_DEG 115
#define SERVO_SWEEP_STEP_PERIOD_MS 35UL

// Conservative SG90 signal range. These are NOT mechanical end stops.
#define SERVO_PULSE_MIN_US 1000
#define SERVO_PULSE_MAX_US 2000

// Arduino ESP32-S3 LEDC: channels 0-3 motors, 4 pump, 6 = independent timer 3.
#define SERVO_PWM_CHANNEL 6
#define SERVO_PWM_FREQUENCY_HZ 50
#define SERVO_PWM_RESOLUTION_BITS 14

// ESP32-S3 LEDC has 8 channels and a maximum 14-bit duty resolution.
#if SERVO_PWM_CHANNEL >= 8 || SERVO_PWM_RESOLUTION_BITS < 1 || SERVO_PWM_RESOLUTION_BITS > 14
#error "ESP32-S3 servo LEDC channel/resolution is out of range"
#endif

#if SERVO_TEST_MIN_DEG < 0 || SERVO_TEST_MAX_DEG > 180 || SERVO_TEST_MIN_DEG > SERVO_TEST_MAX_DEG
#error "Invalid servo calibration angle guard"
#endif

#if SERVO_PULSE_MIN_US < 500 || SERVO_PULSE_MAX_US > 2500 || SERVO_PULSE_MIN_US >= SERVO_PULSE_MAX_US
#error "Invalid servo pulse bounds"
#endif

#if SERVO_CALIBRATION_MODE && SERVO_SWEEP_ENABLED
#error "Calibration mode and live sweep cannot be enabled together"
#endif

#if SERVO_SWEEP_ENABLED && (!SERVO_LIMITS_CONFIRMED || !SERVO_TRAJECTORY_CONFIRMED || \
    SERVO_SWEEP_START_DEG < SERVO_TEST_MIN_DEG || \
    SERVO_SWEEP_START_DEG > SERVO_TEST_MAX_DEG || \
    SERVO_SWEEP_TURN_DEG < SERVO_TEST_MIN_DEG || \
    SERVO_SWEEP_TURN_DEG > SERVO_TEST_MAX_DEG || \
    SERVO_SWEEP_END_DEG < SERVO_TEST_MIN_DEG || \
    SERVO_SWEEP_END_DEG > SERVO_TEST_MAX_DEG || \
    !((SERVO_SWEEP_TURN_DEG < SERVO_SWEEP_START_DEG && SERVO_SWEEP_END_DEG > SERVO_SWEEP_TURN_DEG) || \
      (SERVO_SWEEP_TURN_DEG > SERVO_SWEEP_START_DEG && SERVO_SWEEP_END_DEG < SERVO_SWEEP_TURN_DEG)) || \
    SERVO_SWEEP_STEP_PERIOD_MS < 20)
#error "Confirm safe nozzle limits and sweep time before enabling live servo sweep"
#endif

#endif // SERVO_CONFIG_H
