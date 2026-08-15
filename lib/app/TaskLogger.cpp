#include "TaskManager.h"
#include "RobotConfig.h"
#include "RobotMaster.h"
#include <Arduino.h>

extern RobotMaster robotMaster;

void Task_Logger(void* pvParam) {
    SharedContext* ctx = (SharedContext*)pvParam;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TASK_LOGGER_PERIOD_MS));

        // Doc trang thai motion
        MotionState_t s;
        xSemaphoreTake(ctx->stateMutex, portMAX_DELAY);
        s = ctx->motionState;
        xSemaphoreGive(ctx->stateMutex);

        // Doc lenh dieu khien
        float vx, wz;
        xSemaphoreTake(ctx->cmdMutex, portMAX_DELAY);
        vx = ctx->cmdVel.target_vx;
        wz = ctx->cmdVel.target_wz;
        xSemaphoreGive(ctx->cmdMutex);

        // Im lang phan log chi tiet khi IDLE hoac EMERGENCY (tránh spam log liên tục)
        RobotState_t currentState = robotMaster.getState();
        if (currentState == STATE_IDLE || currentState == STATE_EMERGENCY) {
            // [Finding 7 FIX] Van in log IMU cham (moi ~5 giay) de phat hien drift
            static int idle_log_div = 0;
            if (++idle_log_div >= 50) {
                idle_log_div = 0;
                DBG.printf("[IMU-IDLE] gyro_z:%+.4f rad/s  yaw:%.3f rad (%.1f deg)\n",
                              s.gyro_z, s.theta, s.theta * 180.0f / PI);
            }
            continue;
        }

        // [IMU] Log de phat hien drift khi dang chay
        DBG.printf("[IMU] gyro_z:%+.4f rad/s  yaw:%.3f rad (%.1f deg)\n",
                      s.gyro_z, s.theta, s.theta * 180.0f / PI);

        // Header: hien thi buoc dang chay va trang thai
        DBG.printf("[%s] Vx=%.2f Wz=%.2f\n",
                      (robotMaster.getState() == STATE_EMERGENCY) ? "EMERG" :
                      (TUNING_STEP == 1) ? "STEP1" :
                      (TUNING_STEP == 2) ? "STEP2" :
                      (TUNING_STEP == 3) ? "STEP3" : "STEP4",
                      vx, wz);

        DBG.printf("[FL] Tgt:%5.1f Act:%5.1f Err:%5.1f PWM:%4.0f  ENC:%lld\n",
                      s.tgt_fl, s.w_fl, s.tgt_fl - s.w_fl, s.pwm_fl, s.enc_fl);
        DBG.printf("[RL] Tgt:%5.1f Act:%5.1f Err:%5.1f PWM:%4.0f  ENC:%lld\n",
                      s.tgt_rl, s.w_rl, s.tgt_rl - s.w_rl, s.pwm_rl, s.enc_rl);
        DBG.printf("[FR] Tgt:%5.1f Act:%5.1f Err:%5.1f PWM:%4.0f  ENC:%lld\n",
                      s.tgt_fr, s.w_fr, s.tgt_fr - s.w_fr, s.pwm_fr, s.enc_fr);
        DBG.printf("[RR] Tgt:%5.1f Act:%5.1f Err:%5.1f PWM:%4.0f  ENC:%lld\n",
                      s.tgt_rr, s.w_rr, s.tgt_rr - s.w_rr, s.pwm_rr, s.enc_rr);
        DBG.printf("[ODOM] X:%.3fm Y:%.3fm Theta:%.2frad (%.1fdeg) alpha=%.3f\n",
                      s.x, s.y, s.theta, s.theta * 180.0f / PI, s.alpha);

        // Hien thi du lieu cam bien tu WROOM (neu co)
        bool dataValid;
        SensorPacket_t sd;
        xSemaphoreTake(ctx->stateMutex, portMAX_DELAY);
        dataValid = ctx->sensorDataValid;
        if (dataValid) {
            sd = ctx->lastSensorData;
        }
        xSemaphoreGive(ctx->stateMutex);

        if (dataValid) {
            DBG.printf("[ENV] Fire:%d%d%d Gas:%.1fppm Temp:%.1f°C Batt:%.2fV\n",
                          (sd.fire_flags >> 2) & 1,
                          (sd.fire_flags >> 1) & 1,
                          sd.fire_flags & 1,
                          sd.gas_ppm, sd.temperature, sd.batt_voltage);
        }
    }
}
