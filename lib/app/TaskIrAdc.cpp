#include "TaskManager.h"
#include "IrAdcDriver.h"
#include "RobotConfig.h"
#include <Arduino.h>

// One complete left/center/right snapshot every 100 ms. Conversion wait
// happens OUTSIDE the I2C mutex, so the 50 Hz IMU task keeps bus priority.
void Task_IrAdc(void* pvParam) {
    SharedContext* ctx = static_cast<SharedContext*>(pvParam);
    IrAdcDriver adc;
    bool present = false;
    uint32_t lastProbeMs = 0;
    uint32_t lastLogMs = 0;
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(100));
        const uint32_t now = millis();

        if (!present && (lastProbeMs == 0 || now - lastProbeMs >= 1000)) {
            lastProbeMs = now;
            if (xSemaphoreTake(ctx->i2cMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                present = adc.probe();
                xSemaphoreGive(ctx->i2cMutex);
            }
        }

        uint16_t raw[3] = {0, 0, 0};
        bool valid = present;
        if (valid) {
            for (uint8_t channel = 0; channel < 3; ++channel) {
                if (xSemaphoreTake(ctx->i2cMutex, pdMS_TO_TICKS(2)) != pdTRUE) {
                    valid = false;
                    break;
                }
                const bool started = adc.startSingleEnded(channel);
                xSemaphoreGive(ctx->i2cMutex);
                if (!started) {
                    valid = false;
                    present = false;
                    break;
                }

                vTaskDelay(pdMS_TO_TICKS(3)); // 860 SPS conversion, no bus lock
                if (xSemaphoreTake(ctx->i2cMutex, pdMS_TO_TICKS(2)) != pdTRUE) {
                    valid = false;
                    break;
                }
                const bool read = adc.readConversion(raw[channel]);
                xSemaphoreGive(ctx->i2cMutex);
                if (!read || (raw[channel] & 0x8000u) != 0) {
                    valid = false;
                    present = false;
                    break;
                }
            }
        }

        xSemaphoreTake(ctx->stateMutex, portMAX_DELAY);
        ctx->irValid = valid;
        if (valid) {
            ctx->irRaw[0] = raw[0];
            ctx->irRaw[1] = raw[1];
            ctx->irRaw[2] = raw[2];
            ctx->irSampleMs = millis();
        }
        xSemaphoreGive(ctx->stateMutex);

        if (now - lastLogMs >= 1000) {
            lastLogMs = now;
            if (valid) {
                DBG.printf("[IR] ADS1115 raw L=%u C=%u R=%u (read-only)\n",
                           (unsigned)raw[0], (unsigned)raw[1], (unsigned)raw[2]);
            } else if (!present) {
                DBG.println("[IR] ADS1115 missing or read failed; IR invalid");
            } else {
                DBG.println("[IR] ADS1115 bus busy; IR invalid");
            }
        }
    }
}
