#include "DataStructs.h"
#include "EncoderDriver.h"
#include "IMUDriver.h"
#include "Kinematics.h"
#include "MotionControl.h"
#include "MotorDriver.h"
#include "Odometry.h"
#include "PIDController.h"
#include "RobotConfig.h"
#include "RobotMaster.h"
#include "TaskManager.h"
#include <Arduino.h>
#include <math.h>

// Doi tuong PID 4 banh — dung thong so tu RobotConfig.h
static PIDController pidFL(PID_KP_ACTIVE, PID_KI_ACTIVE, PID_KD_ACTIVE);
static PIDController pidRL(PID_KP_ACTIVE, PID_KI_ACTIVE, PID_KD_ACTIVE);
static PIDController pidFR(PID_KP_ACTIVE, PID_KI_ACTIVE, PID_KD_ACTIVE);
static PIDController pidRR(PID_KP_ACTIVE, PID_KI_ACTIVE, PID_KD_ACTIVE);

// Tham chieu den RobotMaster (duoc tao trong main.cpp)
extern RobotMaster robotMaster;

void Task_Motion(void *pvParam) {
  SharedContext *ctx = (SharedContext *)pvParam;

  int64_t prev_fl = 0, prev_rl = 0, prev_fr = 0, prev_rr = 0;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(TASK_MOTION_PERIOD_MS);
  const float dt = (float)TASK_MOTION_PERIOD_MS / 1000.0f;

  // Trang thai Ramp
  float ramp_vx = 0.0f;
  float ramp_wz = 0.0f;

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    // ========================================================
    // 1. DOC SENSOR & CAP NHAT ODOMETRY
    // ========================================================
    int64_t curr_fl = encoderDriver.getCountFL();
    int64_t curr_rl = encoderDriver.getCountRL();
    int64_t curr_fr = encoderDriver.getCountFR();
    int64_t curr_rr = encoderDriver.getCountRR();

    const float ticks2rads = (TWO_PI / (float)ENCODER_TICKS_PER_REV) / dt;
    float w_fl = (curr_fl - prev_fl) * ticks2rads;
    float w_rl = (curr_rl - prev_rl) * ticks2rads;
    float w_fr = (curr_fr - prev_fr) * ticks2rads;
    float w_rr = (curr_rr - prev_rr) * ticks2rads;
    prev_fl = curr_fl;
    prev_rl = curr_rl;
    prev_fr = curr_fr;
    prev_rr = curr_rr;

    float gyro_z = imuDriver.getGyroZ();
    float v_left = (w_fl + w_rl) * 0.5f * WHEEL_RADIUS_M;
    float v_right = (w_fr + w_rr) * 0.5f * WHEEL_RADIUS_M;
    float v_x = (v_left + v_right) / 2.0f;
    float wz_encoder = (v_right - v_left) / TRACK_WIDTH_M;

    odometry.update(v_x, wz_encoder, gyro_z, dt);
    float current_theta = odometry.getOdometry().yaw;
    float current_x = odometry.getOdometry().x;
    float current_y = odometry.getOdometry().y;
    float current_alpha = odometry.getLastAlpha();

    // ========================================================
    // 2. DOC LENH DIEU KHIEN (Mutex protected)
    //    Slave chi nhan (vx, wz) tu micro-ROS /cmd_vel hoac Web
    // ========================================================
    float cmd_vx, cmd_wz;
    xSemaphoreTake(ctx->cmdMutex, portMAX_DELAY);
    cmd_vx = ctx->cmdVel.target_vx;
    cmd_wz = ctx->cmdVel.target_wz;
    xSemaphoreGive(ctx->cmdMutex);

    // ========================================================
    // 4. KIEM TRA STATE MACHINE
    // ========================================================
    if (!robotMaster.isMotionAllowed()) {
      // STATE_IDLE hoac STATE_EMERGENCY -> dung motor
      motorDriver.stopAll();
      pidFL.reset();
      pidRL.reset();
      pidFR.reset();
      pidRR.reset();
      ramp_vx = 0.0f;
      ramp_wz = 0.0f;

      // Van cap nhat state de Logger doc
      xSemaphoreTake(ctx->stateMutex, portMAX_DELAY);
      ctx->motionState = {0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          curr_fl,
                          curr_rl,
                          curr_fr,
                          curr_rr,
                          current_theta,
                          current_x,
                          current_y,
                          current_alpha,
                          gyro_z};
      ctx->odom = odometry.getOdometry();
      xSemaphoreGive(ctx->stateMutex);
      continue;
    }

    // ========================================================
    // 5. AP DUNG RAMP (Tang/giam toc mem)
    // ========================================================
    ramp_vx =
        MotionControl::applyRamp(ramp_vx, cmd_vx, (float)RAMP_ACCEL_VX, dt);
    ramp_wz =
        MotionControl::applyRamp(ramp_wz, cmd_wz, (float)RAMP_ACCEL_WZ, dt);

    // ========================================================
    // 6. XU LY STOP & IDLE
    // ========================================================
    const bool ramped_stopped =
        (fabsf(ramp_vx) < 0.01f && fabsf(ramp_wz) < 0.01f);
    if (cmd_vx == 0.0f && cmd_wz == 0.0f && ramped_stopped) {
      motorDriver.stopAll();
      pidFL.reset();
      pidRL.reset();
      pidFR.reset();
      pidRR.reset();
      ramp_vx = 0.0f;
      ramp_wz = 0.0f;

      xSemaphoreTake(ctx->stateMutex, portMAX_DELAY);
      ctx->motionState = {0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          curr_fl,
                          curr_rl,
                          curr_fr,
                          curr_rr,
                          current_theta,
                          current_x,
                          current_y,
                          current_alpha,
                          gyro_z};
      ctx->odom = odometry.getOdometry();
      xSemaphoreGive(ctx->stateMutex);
      continue;
    }

    // ========================================================
    // 7. TINH MUC TIEU + PWM
    // ========================================================
    MotorSpeeds_t tgt = kinematics.computeWheelSpeeds(ramp_vx, ramp_wz);
    float active_offset = MotionControl::selectOffset(cmd_vx, cmd_wz);

    float pwm_fl =
        MotionControl::computePWM(tgt.v_fl, w_fl, pidFL, active_offset, dt);
    float pwm_rl =
        MotionControl::computePWM(tgt.v_rl, w_rl, pidRL, active_offset, dt);
    float pwm_fr =
        MotionControl::computePWM(tgt.v_fr, w_fr, pidFR, active_offset, dt);
    float pwm_rr =
        MotionControl::computePWM(tgt.v_rr, w_rr, pidRR, active_offset, dt);

    motorDriver.setSpeeds((int)pwm_fl, (int)pwm_rl, (int)pwm_fr, (int)pwm_rr);

    // ========================================================
    // 8. CAP NHAT TRANG THAI CHO LOGGER
    // ========================================================
    xSemaphoreTake(ctx->stateMutex, portMAX_DELAY);
    ctx->motionState = {w_fl,     w_rl,          w_fr,      w_rr,     tgt.v_fl,
                        tgt.v_rl, tgt.v_fr,      tgt.v_rr,  pwm_fl,   pwm_rl,
                        pwm_fr,   pwm_rr,        curr_fl,   curr_rl,  curr_fr,
                        curr_rr,  current_theta, current_x, current_y,
                        current_alpha, gyro_z};
    ctx->odom = odometry.getOdometry();
    xSemaphoreGive(ctx->stateMutex);
  }
}
