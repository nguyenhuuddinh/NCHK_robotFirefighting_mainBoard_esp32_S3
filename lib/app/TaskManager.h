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

/** Task micro-ROS communication (Core 0, Priority 3) — legacy, giữ để compile rollback */
void Task_MicroROS(void* pvParam);

/** Task raw serial V2 communication (Core 0, Priority 3) — thay thế Task_MicroROS */
void Task_SerialComm(void* pvParam);

/** Task in debug log (Core 0, Priority thap) */
void Task_Logger(void* pvParam);

// Bat macro nay neu can fallback Web Server de test motor nhanh (tat di de uu tien micro-ROS)
// #define ENABLE_WEB_FALLBACK

/** Task WebServer (Core 0) - [Finding 3] Giu lai fallback nhung disable de giam tai SLAM */
#ifdef ENABLE_WEB_FALLBACK
void Task_WebServer(void* pvParam);
#endif

/** Task nhan du lieu tu WROOM qua UART + gui lenh actuator (Core 0) */
void Task_UART(void* pvParam);

// ============================================================
// TASKMANAGER API
// ============================================================
void TaskManager_init();
SharedContext* TaskManager_getContext();
