#include "TaskManager.h"
#include "RobotConfig.h"
#include <Arduino.h>

// Context toan cuc — tao 1 lan, ton tai suot doi song chuong trinh
static SharedContext g_ctx;

SharedContext* TaskManager_getContext() {
    return &g_ctx;
}

void TaskManager_init() {
    // 1. Tao Mutex TRUOC khi bat ky Task nao chay
    g_ctx.cmdMutex = xSemaphoreCreateMutex();
    g_ctx.stateMutex = xSemaphoreCreateMutex();
    g_ctx.i2cMutex = xSemaphoreCreateMutex();

    if (g_ctx.cmdMutex == nullptr || g_ctx.stateMutex == nullptr ||
        g_ctx.i2cMutex == nullptr) {
        DBG.println("[ERROR] Failed to create Mutex! System halted.");
        while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    // 2. Tao Queue cho UART
    //    sensorQueue: chi giu 1 item moi nhat (xQueueOverwrite)
    //    actuatorQueue: giu 1 item moi nhat
    g_ctx.sensorQueue = xQueueCreate(1, sizeof(SensorPacket_t));
    g_ctx.actuatorQueue = xQueueCreate(1, sizeof(ActuatorCmd_t));

    if (g_ctx.sensorQueue == nullptr || g_ctx.actuatorQueue == nullptr) {
        DBG.println("[ERROR] Failed to create Queue! System halted.");
        while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    // 3. Khoi tao gia tri mac dinh
    g_ctx.cmdVel = {};
    g_ctx.lastCmdReceivedMs = 0;
    g_ctx.hasValidCmd = false;
    g_ctx.motionState = {};
    g_ctx.imuValid = false;
    g_ctx.imuReadFailures = 0;
    g_ctx.imuBusContentions = 0;
    g_ctx.lastSensorData = {};
    g_ctx.sensorDataValid = false;
    g_ctx.irRaw[0] = g_ctx.irRaw[1] = g_ctx.irRaw[2] = 0;
    g_ctx.irSampleMs = 0;
    g_ctx.irValid = false;

    // 4. Tao cac FreeRTOS Task
    // Core 1: Motion (Real-time, Priority cao nhat)
    xTaskCreatePinnedToCore(
        Task_Motion, "Task_Motion",
        TASK_MOTION_STACK, &g_ctx,
        TASK_MOTION_PRIORITY, nullptr, CORE_1
    );

    // One Core-0 owner for pump/servo PWM. Higher priority than Serial so a
    // stalled serial task cannot suspend the 750 ms lease or 5 s run cap.
    const BaseType_t actuatorCreated = xTaskCreatePinnedToCore(
        Task_Actuator, "Task_Actuator",
        4096, &g_ctx, TASK_SERIALCOMM_PRIORITY + 1, nullptr, CORE_0
    );
    if (actuatorCreated != pdPASS) {
        // PWM stays OFF after setup when its sole owner could not start.
        DBG.println("[ERROR] Actuator owner task creation failed; pump stays OFF.");
    }

    // Core 0: Raw Serial V2 Communication (Priority 3)
    // Thay thế Task_MicroROS trong runtime chính.
    // Task_MicroROS source giữ nguyên trong lib/app/ để rollback.
    xTaskCreatePinnedToCore(
        Task_SerialComm, "Task_Serial",
        TASK_SERIALCOMM_STACK, &g_ctx,
        TASK_SERIALCOMM_PRIORITY, nullptr, CORE_0
    );

    // Chỉ đọc ADC trên Core 0. Không tạo lệnh chuyển động/chấp hành.
    xTaskCreatePinnedToCore(
        Task_IrAdc, "Task_IrAdc",
        3072, &g_ctx, 2, nullptr, CORE_0
    );

    // [Finding 3 FIX] Core 0: Web Server (Fallback)
    // Dung ENABLE_WEB_FALLBACK (dinh nghia trong TaskManager.h) de bat/tat.
    // Hien tai dang tat de giam tai Core 0 cho SLAM/serial.
#ifdef ENABLE_WEB_FALLBACK
    xTaskCreatePinnedToCore(
        Task_WebServer, "Task_WebSrv",
        4096, &g_ctx,
        2, nullptr, CORE_0
    );
#endif

    // Core 0: UART Rx/Tx voi WROOM (tuy chon)
#if ENABLE_WROOM_UART
    xTaskCreatePinnedToCore(
        Task_UART, "Task_UART",
        TASK_UART_STACK, &g_ctx,
        TASK_UART_PRIORITY, nullptr, CORE_0
    );
#endif

    // Core 0: Logger (Debug)
    xTaskCreatePinnedToCore(
        Task_Logger, "Task_Logger",
        TASK_LOGGER_STACK, &g_ctx,
        TASK_LOGGER_PRIORITY, nullptr, CORE_0
    );

    DBG.println("[OK] TaskManager: All tasks created successfully.");
    DBG.printf("[CFG] Motion: Core%d Pri%d | Serial: Core%d Pri%d Stack%d\n",
                  CORE_1, TASK_MOTION_PRIORITY,
                  CORE_0, TASK_SERIALCOMM_PRIORITY, TASK_SERIALCOMM_STACK);
#if ENABLE_WROOM_UART
    DBG.printf("[CFG] WROOM UART: Core%d Pri%d | Logger: Core%d Pri%d\n",
                  CORE_0, TASK_UART_PRIORITY, CORE_0, TASK_LOGGER_PRIORITY);
#else
    DBG.printf("[CFG] WROOM UART: disabled | Logger: Core%d Pri%d\n",
                  CORE_0, TASK_LOGGER_PRIORITY);
#endif
}
