/**
 * @file main.cpp
 * @brief ESP32-S3 Motion Slave - Entry Point
 *
 * Kiem truc FreeRTOS Multi-Task:
 *   Core 1: Task_Motion      (50Hz)  — Encoder, IMU, PID, PWM
 *   Core 0: Task_SerialComm  (50Hz)  — Raw Serial V2 (USB CDC -> Pi)
 *   Core 0: Task_UART        (20Hz)  — Tuy chon; mac dinh co lap WROOM
 *   Core 0: Task_Logger      (5Hz)   — Debug Serial Monitor
 *
 * Serial ports:
 *   Serial  (USB CDC) → Raw Serial V2 transport (Raspberry Pi)
 *   Serial0 (UART0/CH340) → Debug log + Serial Monitor (Laptop)
 *   Serial2 (UART2) → Tuy chon; chi bat sau khi remap khoi GPIO19/20
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
#include "ServoConfig.h"
#include "ServoOutput.h"
#include "PinConfig.h"
#include <stdlib.h>
#include <string.h>

#if !SERVO_CALIBRATION_MODE
#include "RobotMaster.h"

// Driver instances (tao boi cac file .cpp trong lib/driver/)
#include "EncoderDriver.h"
#include "IMUDriver.h"
#include "MotorDriver.h"
#include "PumpOutput.h"
#include "SlaveComm.h"

// TaskManager — tao va khoi dong tat ca FreeRTOS tasks
extern void TaskManager_init();

// RobotMaster — may trang thai chinh (dung global de cac Task truy cap)
RobotMaster robotMaster;
#endif

#if SERVO_CALIBRATION_MODE
static char s_cal_line[24];
static size_t s_cal_len = 0;
static uint32_t s_cal_last_angle_ms = 0;

static void servoCalCommand(const char* line) {
    if (strcmp(line, "off") == 0) {
        servoOutput.stop();
        DBG.println("[SERVO-CAL] OFF");
        return;
    }
    if (strcmp(line, "help") == 0) {
        DBG.printf("[SERVO-CAL] a <angle> (%d..%d), off, help; auto-off after %lu ms\n",
                   SERVO_TEST_MIN_DEG, SERVO_TEST_MAX_DEG,
                   static_cast<unsigned long>(SERVO_CAL_HOLD_MAX_MS));
        return;
    }
    if (line[0] == 'a' && line[1] == ' ') {
        char* end = nullptr;
        const long angle = strtol(line + 2, &end, 10);
        while (*end == ' ') ++end;
        if (*end == '\0' && angle >= SERVO_TEST_MIN_DEG &&
            angle <= SERVO_TEST_MAX_DEG) {
            if (servoOutput.writeAngle(angle)) {
                s_cal_last_angle_ms = millis();
                DBG.printf("[SERVO-CAL] angle=%ld deg\n", angle);
            } else {
                servoOutput.stop();
                DBG.println("[ERROR][SERVO-CAL] Servo PWM init failed; GPIO38 OFF.");
            }
            return;
        }
    }
    servoOutput.stop();
    DBG.println("[SERVO-CAL] Rejected. Type help; no angle sent.");
}

void setup() {
    // Isolated firmware: no USB-CDC bridge, motor, pump, IMU or FreeRTOS tasks.
    // Keep all existing motor-enable outputs and the pump PWM explicitly LOW;
    // this calibration build never initializes their PWM peripherals.
    const uint8_t safe_low_pins[] = {
        PUMP_PWM_PIN, MOT_ENA_FL_PIN, MOT_ENB_RL_PIN,
        MOT_ENA_FR_PIN, MOT_ENB_RR_PIN};
    for (uint8_t pin : safe_low_pins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    DBG.begin(115200);
    servoOutput.init();
    DBG.println("[SERVO-CAL] GPIO38 isolated calibration. Servo is OFF at boot.");
    DBG.printf("[SERVO-CAL] LEDC channel=%d freq=%dHz resolution=%d-bit\n",
               SERVO_PWM_CHANNEL, SERVO_PWM_FREQUENCY_HZ,
               SERVO_PWM_RESOLUTION_BITS);
    servoCalCommand("help");
}

void loop() {
    uint8_t budget = 32;
    while (budget-- > 0 && DBG.available() > 0) {
        const char c = static_cast<char>(DBG.read());
        if (c == '\r' || c == '\n') {
            if (s_cal_len > 0) {
                s_cal_line[s_cal_len] = '\0';
                servoCalCommand(s_cal_line);
                s_cal_len = 0;
            }
        } else if (s_cal_len < sizeof(s_cal_line) - 1) {
            s_cal_line[s_cal_len++] = c;
        } else {
            s_cal_len = 0;
            servoOutput.stop();
            DBG.println("[SERVO-CAL] Line too long; discarded.");
        }
    }
    if (servoOutput.active() &&
        static_cast<uint32_t>(millis() - s_cal_last_angle_ms) >= SERVO_CAL_HOLD_MAX_MS) {
        servoOutput.stop();
        DBG.println("[SERVO-CAL] Hold timeout -> OFF");
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}

#else
void setup() {
    // Khoi tao CA HAI cong Serial:
    //   Serial  (USB CDC) = Raw Serial V2 transport → Raspberry Pi
    //   Serial0 (UART0/CH340) = Debug log → Laptop Serial Monitor
    Serial.setRxBufferSize(4096);  // Tăng bộ đệm nhận USB CDC từ 256 lên 4096 bytes
    // [F2-L13-1] USBCDC không có setTxBufferSize. Fixed 64 byte.
    Serial.setTxTimeoutMs(0);      // Đảm bảo hàm write() trả về ngay lập tức (non-blocking)
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
    DBG.printf("[CFG] TICKS/REV=%d | R=%.3fm | TRACK=%.3fm\n",
                  (int)ENCODER_TICKS_PER_REV, WHEEL_RADIUS_M, TRACK_WIDTH_M);
    DBG.println("[CFG] Serial=USB-CDC(V2) | Serial0=UART(Debug)");


    // 1. Khoi tao phan cung
    motorDriver.init();
    if (pumpOutput.init()) {
        DBG.println("[OK] Pump PWM GPIO47 initialized OFF.");
    } else {
        DBG.println("[ERROR] Pump PWM GPIO47 init failed; output remains OFF.");
    }
#if SERVO_SWEEP_ENABLED
    servoOutput.init();
    DBG.printf("[CFG] Servo sweep %d -> %d -> %d deg, 1 deg per >=%lu ms; gated by pump PWM enable.\n",
               SERVO_SWEEP_START_DEG, SERVO_SWEEP_TURN_DEG, SERVO_SWEEP_END_DEG,
               static_cast<unsigned long>(SERVO_SWEEP_STEP_PERIOD_MS));
#endif
    encoderDriver.init();
    if (imuDriver.init()) {
        DBG.println("[OK] IMU MPU6050 Initialized.");
    } else {
        DBG.println("[WARN] MPU6050 Init Failed! Fallback to encoder-only.");
    }
#if ENABLE_WROOM_UART
    slaveComm.init();
#else
    DBG.println("[CFG] WROOM UART disabled; GPIO19/GPIO20 reserved for native USB");
#endif

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
#endif // SERVO_CALIBRATION_MODE
