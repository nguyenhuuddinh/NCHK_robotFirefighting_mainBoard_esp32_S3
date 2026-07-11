#pragma once

#include "DataStructs.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// ============================================================
// DU LIEU CHIA SE GIUA CAC TASK (Shared Context)
// Duoc tao boi TaskManager, truyen vao cac Task qua pvParam
// ============================================================
struct SharedContext {
    // --- Mutex ---
    SemaphoreHandle_t cmdMutex;    // Bao ve cmdVel
    SemaphoreHandle_t stateMutex;  // Bao ve motionState

    // --- Queue ---
    QueueHandle_t sensorQueue;     // SensorPacket_t tu WROOM (xQueueOverwrite)
    QueueHandle_t actuatorQueue;   // ActuatorCmd_t gui xuong WROOM

    // --- Lenh dieu khien (Core0 ghi, Core1 doc) ---
    CmdVel_t cmdVel;

    // --- Trang thai motion (Core1 ghi, Core0 doc) ---
    MotionState_t motionState;
    OdometryData_t odom;

    // --- Du lieu cam bien moi nhat (doc boi Logger/MicroROS) ---
    SensorPacket_t lastSensorData;
    bool sensorDataValid;          // true khi da nhan duoc it nhat 1 packet
};

// ============================================================
// FORWARD DECLARE CAC TASK FUNCTIONS
// ============================================================

/** Task dieu khien chuyen dong (Core 1, Priority cao nhat) */
void Task_Motion(void* pvParam);

/** Task micro-ROS communication (Core 0, Priority 3) */
void Task_MicroROS(void* pvParam);

/** Task xu ly Web Server (Core 0 — tam, se thay bang micro-ROS) */
void Task_WebServer(void* pvParam);

/** Task in debug log (Core 0, Priority thap) */
void Task_Logger(void* pvParam);

/** Task nhan du lieu tu WROOM qua UART + gui lenh actuator (Core 0) */
void Task_UART(void* pvParam);

// ============================================================
// TASKMANAGER API
// ============================================================
void TaskManager_init();
SharedContext* TaskManager_getContext();
