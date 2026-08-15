#include "TaskManager.h"
#include "RobotConfig.h"
#include "SlaveComm.h"
#include <Arduino.h>

/**
 * @brief Task UART: Giao tiep voi ESP32-WROOM.
 *
 * Chuc nang:
 *   1. Doc SensorPacket_t tu WROOM (fire, gas, temp, batt)
 *   2. Gui ActuatorCmd_t xuong WROOM (servo pan, bom, buzzer)
 *
 * Chay tren Core 0, chu ky 50ms (20Hz).
 */
void Task_UART(void* pvParam) {
    SharedContext* ctx = (SharedContext*)pvParam;
    uint32_t lastStatsMs = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TASK_UART_PERIOD_MS));

        // ========================================================
        // 1. NHAN SENSOR DATA TU WROOM
        // ========================================================
        SensorPacket_t packet;
        while (slaveComm.tryReceive(packet)) {
            // Luu packet moi nhat vao context (cho Logger va micro-ROS doc)
            xSemaphoreTake(ctx->stateMutex, portMAX_DELAY);
            ctx->lastSensorData = packet;
            ctx->sensorDataValid = true;
            xSemaphoreGive(ctx->stateMutex);
        }

        // ========================================================
        // 2. GUI LENH ACTUATOR XUONG WROOM (neu co lenh moi)
        // ========================================================
        ActuatorCmd_t cmd;
        if (xQueueReceive(ctx->actuatorQueue, &cmd, 0) == pdTRUE) {
            slaveComm.sendCommand(cmd.servo_pan, cmd.pump_on, cmd.buzzer_on);
        }

        // ========================================================
        // 3. IN THONG KE MOI 10 GIAY (debug)
        // ========================================================
        uint32_t now = millis();
        if ((now - lastStatsMs) > 10000) {
            lastStatsMs = now;
            uint32_t ok = slaveComm.getPacketsOk();
            uint32_t fail = slaveComm.getPacketsFail();
            uint32_t total = ok + fail;
            if (total > 0) {
                float rate = 100.0f * (float)ok / (float)total;
                DBG.printf("[UART] Packets: %lu ok, %lu fail (%.1f%%)\n",
                              ok, fail, rate);
            } else {
                DBG.println("[UART] No WROOM data (0 packets received)");
            }
        }
    }
}
