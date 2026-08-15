/**
 * @file main.cpp
 * @brief ESP32-S3 Motion Slave - Entry Point
 *
 * Kiem truc FreeRTOS Multi-Task:
 *   Core 1: Task_Motion      (50Hz)  — Encoder, IMU, PID, PWM
 *   Core 0: Task_SerialComm  (50Hz)  — Raw Serial V2 (USB CDC -> Pi)
 *   Core 0: Task_UART        (20Hz)  — Giao tiep UART voi ESP32-WROOM
 *   Core 0: Task_Logger      (5Hz)   — Debug Serial Monitor
 *
 * Serial ports:
 *   Serial  (USB CDC) → Raw Serial V2 transport (Raspberry Pi)
 *   Serial0 (UART0/CH340) → Debug log + Serial Monitor (Laptop)
 *   Serial2 (UART2) → Giao tiep WROOM
 *
 * Legacy: Task_MicroROS source còn trong lib/app/ để rollback.
 *         micro_ros_platformio dependency giữ nguyên trong platformio.ini.
 *
 * Toan bo logic da duoc tach vao lib/:
 *   lib/common/  — DataStructs, PinConfig, RobotConfig
 *   lib/driver/  — MotorDriver, EncoderDriver, IMUDriver, SlaveComm
 *   lib/service/ — Kinematics, Odometry, PIDController, MotionControl, SerialComm
 *   lib/app/     — RobotMaster, TaskManager, Task*.cpp
 */

#include <Arduino.h>
#include "RobotConfig.h"
#include "RobotMaster.h"

// Driver instances (tao boi cac file .cpp trong lib/driver/)
#include "EncoderDriver.h"
#include "IMUDriver.h"
#include "MotorDriver.h"
#include "SlaveComm.h"

// TaskManager — tao va khoi dong tat ca FreeRTOS tasks
extern void TaskManager_init();

// RobotMaster — may trang thai chinh (dung global de cac Task truy cap)
RobotMaster robotMaster;

void setup() {
    // Khoi tao CA HAI cong Serial:
    //   Serial  (USB CDC) = Raw Serial V2 transport → Raspberry Pi
    //   Serial0 (UART0/CH340) = Debug log → Laptop Serial Monitor
    Serial.setRxBufferSize(4096);  // Tăng bộ đệm nhận USB CDC từ 256 lên 4096 bytes
    Serial.setTxBufferSize(256);   // [F2-L2-1] Tăng TX buffer > 192 bytes (mặc định TinyUSB là 64 bytes)
    Serial.setTxTimeoutMs(0);      // [F2-L2-1] Đảm bảo hàm write() trả về ngay lập tức (non-blocking)
    Serial.begin(115200);   // USB CDC cho Raw Serial V2
    DBG.begin(115200);      // UART0/CH340 cho debug
    vTaskDelay(pdMS_TO_TICKS(800)); // Cho Serial on dinh

    DBG.println("\n======================================");
    DBG.println("  ESP32-S3 MOTION SLAVE - FreeRTOS");
    DBG.println("  Raw Serial V2 (USB CDC -> Pi)");
    DBG.println("======================================");
    DBG.printf("[CFG] TUNING_STEP %d | Kp=%.1f Ki=%.1f Kd=%.1f\n",
                  TUNING_STEP, (float)PID_KP_ACTIVE, (float)PID_KI_ACTIVE,
                  (float)PID_KD_ACTIVE);
    DBG.printf("[CFG] FF_offset=%.0f FF_offset_turn=%.0f K_ff=%.1f\n",
                  (float)FF_PWM_OFFSET, (float)FF_PWM_OFFSET_TURN,
                  (float)FF_K_FF_ACTIVE);
    DBG.printf("[CFG] TICKS/REV=%d | R=%.3fm | TRACK=%.2fm\n",
                  (int)ENCODER_TICKS_PER_REV, WHEEL_RADIUS_M, TRACK_WIDTH_M);
    DBG.println("[CFG] Serial=USB-CDC(V2) | Serial0=UART(Debug)");


    // 1. Khoi tao phan cung
    motorDriver.init();
    encoderDriver.init();
    if (imuDriver.init()) {
        DBG.println("[OK] IMU MPU6050 Initialized.");
    } else {
        DBG.println("[WARN] MPU6050 Init Failed! Fallback to encoder-only.");
    }
    slaveComm.init();

    // 2. Khoi tao State Machine
    robotMaster.init();

    // 3. Khoi tao va start tat ca FreeRTOS Tasks
    TaskManager_init();

    DBG.println("[OK] System ready.");
}

// loop() chay tren Core 1 priority 1 (thap hon Task_Motion)
// Khong dung loop() de xu ly bat ky logic nao
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}