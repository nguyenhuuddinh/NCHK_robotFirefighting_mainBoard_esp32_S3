/**
 * @file TaskMicroROS.cpp
 * @brief FreeRTOS Task cho micro-ROS (Core 0)
 *
 * Chay tren Core 0, Priority 3 (sau Task_Motion, truoc UART va Logger)
 * Goi MicroRosComm::spinOnce() moi 20ms (50Hz)
 *
 * Task nay SE THAY THE Task_WebServer khi Phase 4 hoan thanh (Buoc 4.10)
 * Hien tai chay SONG SONG voi WebServer de test.
 *
 * Stack: 8192 bytes (micro-ROS can nhieu hon binh thuong)
 */

#include "TaskManager.h"
#include "MicroRosComm.h"
#include "RobotConfig.h"
#include <Arduino.h>

// Instance MicroRosComm (cap phat tinh, ton tai suot doi song chuong trinh)
static MicroRosComm g_microRos;

// Accessor cho cac module khac can truy cap (Phase 4.3+)
MicroRosComm* getMicroRosComm() {
    return &g_microRos;
}

void Task_MicroROS(void* pvParam) {
    // SharedContext* ctx = static_cast<SharedContext*>(pvParam);

    // Cho he thong on dinh truoc khi init micro-ROS
    vTaskDelay(pdMS_TO_TICKS(2000));

    DBG.println("[uROS] Task_MicroROS started on Core 0");

    // Khoi tao transport
    if (!g_microRos.init()) {
        DBG.println("[uROS][ERROR] Transport init failed! Task suspended.");
        vTaskSuspend(nullptr);
    }

    DBG.println("[uROS] Waiting for micro-ROS Agent...");

    // === Main Loop ===
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        // Spin micro-ROS state machine (WAITING → CONNECTED ↔ DISCONNECTED)
        g_microRos.spinOnce();

        // [QA CRITICAL FIX] Khi dang cho Agent (WAITING/DISCONNECTED), rmw_uros_ping_agent tốn 100-200ms.
        // Neu dung vTaskDelayUntil(20ms), Task se khong bao gio duoc ngu (do T_exec > T_period),
        // dan den chiem dung 100% Core 0 (Priority 3) -> IDLE0 bi chet doi -> Task Watchdog Reset xe!
        // -> Khi chua ket noi: Ping 2Hz (ngu 500ms) la du va an toan cho WDT.
        // -> Khi da ket noi: Chay 50Hz (20ms) bang vTaskDelayUntil de publish mượt.
        if (g_microRos.getState() != UROS_CONNECTED) {
            vTaskDelay(pdMS_TO_TICKS(500));
            xLastWake = xTaskGetTickCount(); // Reset lai mốc thời gian trước khi vào lại 50Hz
        } else {
            g_microRos.publishData();
            vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(TASK_MICROROS_PERIOD_MS));
        }
    }
}
