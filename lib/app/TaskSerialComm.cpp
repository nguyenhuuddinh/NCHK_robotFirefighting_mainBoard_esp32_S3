/**
 * @file TaskSerialComm.cpp
 * @brief FreeRTOS Task cho Raw Serial V2 Communication (Core 0)
 *
 * Thay thế Task_MicroROS trong runtime chính.
 * Chạy trên Core 0, Priority 3, chu kỳ 20ms.
 *
 * TX (ESP32 -> Pi):
 *   STATE @ 10Hz (mỗi 100ms): odom + motion data
 *   ENV   @ 2Hz  (mỗi 500ms): sensor data từ WROOM
 *
 * RX (Pi -> ESP32):
 *   CMD:  cập nhật cmdVel + notify watchdog (duplicate bị bỏ qua)
 *   FIRE: cập nhật servo pan qua actuatorQueue
 *   PUMP: cập nhật pump on/off qua actuatorQueue
 *
 * Protocol: @PAYLOAD*CCCC\n (CRC16-CCITT-FALSE)
 * Transport: Serial USB CDC native
 * Debug: DBG/Serial0 (UART0/CH340)
 * KHÔNG log lên Serial USB CDC.
 */

#include "TaskManager.h"
#include "SerialComm.h"
#include "RobotConfig.h"
#include "RobotMaster.h"
#include <Arduino.h>
#include <math.h>

extern RobotMaster robotMaster;

// ============================================================
// STATIC INSTANCES (tồn tại suốt đời sống chương trình)
// ============================================================
static SerialParser   g_parser;
static SerialCommTelemetry g_telemetry;

// [F2-3] TX sequence counter CHUNG cho cả STATE và ENV
// Mỗi hướng truyền có MỘT bộ đếm. Tăng sau mỗi frame được tạo.
static uint32_t g_txSeq = 0;

// Trạng thái actuator tĩnh (giống pattern trong MicroRosComm callbacks)
static uint8_t s_servo_pan = 90;
static uint8_t s_pump_on   = 0;
static uint8_t s_buzzer_on = 0;

// TX buffer tĩnh — không malloc
static char g_txBuf[SERIAL_BUFFER_SIZE];
static char g_payloadBuf[SERIAL_BUFFER_SIZE];

// [F2-5] USB CDC reconnect detection
// Trên ESP32-S3 USB CDC native, Serial operator bool trả true khi host đã enumerate.
// Khi host rút USB hoặc tắt port, operator bool trả false.
// Limitation: một số trường hợp host có thể không báo DTR/RTS đúng.
//             Đây là cơ chế nhỏ nhất có sẵn trong Arduino ESP32-S3 CDC.
static bool g_lastHostConnected = false;

// ============================================================
// HELPER: Xử lý một frame RX đã parse xong
// [F2-1] Duplicate CMD không cập nhật cmdVel và không reset watchdog.
// ============================================================
static void handleRxFrame(const SerialRxResult& rx, SharedContext* ctx) {
    // [F2-L2-3] Bỏ qua toàn bộ lệnh duplicate (CMD/FIRE/PUMP)
    if (rx.is_duplicate) {
        return;
    }

    switch (rx.type) {

    case RX_CMD: {
        // CMD: cập nhật cmdVel dưới cmdMutex và notify watchdog
        xSemaphoreTake(ctx->cmdMutex, portMAX_DELAY);
        ctx->cmdVel.target_vx = rx.f1;
        ctx->cmdVel.target_wz = rx.f2;
        xSemaphoreGive(ctx->cmdMutex);

        // Reset watchdog — CHỈ CMD hợp lệ không duplicate mới reset
        robotMaster.notifyCmdReceived();
        break;
    }

    case RX_FIRE: {
        // FIRE: x -> servo pan [0, 180]
        // Giữ behavior hiện tại: x ánh xạ sang servo pan 0..180
        float pan_f = 90.0f + (rx.f1 * 90.0f);
        if (pan_f > 180.0f) pan_f = 180.0f;
        if (pan_f < 0.0f) pan_f = 0.0f;

        s_servo_pan = (uint8_t)pan_f;
        DBG.printf("[ACT] Servo Pan -> %d do\n", s_servo_pan);

        // xQueueOverwrite: lệnh mới luôn ghi đè lệnh cũ
        if (ctx->actuatorQueue != nullptr) {
            ActuatorCmd_t cmd = {ACTUATOR_CMD_HEADER, s_servo_pan, s_pump_on, s_buzzer_on, 0};
            xQueueOverwrite(ctx->actuatorQueue, &cmd);
        }
        break;
    }

    case RX_PUMP: {
        // PUMP: on/off
        s_pump_on = rx.pump;
        DBG.printf("[ACT] Pump %s\n", s_pump_on ? "ON" : "OFF");

        // xQueueOverwrite: đảm bảo pump OFF không bao giờ bị drop
        if (ctx->actuatorQueue != nullptr) {
            ActuatorCmd_t cmd = {ACTUATOR_CMD_HEADER, s_servo_pan, s_pump_on, s_buzzer_on, 0};
            xQueueOverwrite(ctx->actuatorQueue, &cmd);
        }
        break;
    }

    default:
        break;
    }
}

// ============================================================
// TASK ENTRY POINT
// ============================================================
void Task_SerialComm(void* pvParam) {
    SharedContext* ctx = static_cast<SharedContext*>(pvParam);

    // Chờ hệ thống ổn định
    vTaskDelay(pdMS_TO_TICKS(2000));

    DBG.println("[SERIAL] Task_SerialComm started on Core 0");
    DBG.println("[SERIAL] Protocol V2: Raw Serial USB CDC");

    // Reset parser
    g_parser.reset();

    // Divider cho TX: tại 20ms period
    //   STATE @ 10Hz = mỗi 5 cycles (100ms)
    //   ENV   @ 2Hz  = mỗi 25 cycles (500ms)
    int stateDivider = 0;
    int envDivider   = 0;

    // Divider cho telemetry log (mỗi 10 giây)
    uint32_t lastTelemetryMs = millis();

    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(TASK_SERIALCOMM_PERIOD_MS));

        // ========================================================
        // 0. CẬP NHẬT STATE MACHINE (Watchdog check)
        //    Giống TaskMicroROS: gọi robotMaster.update() đều đặn
        // ========================================================
        robotMaster.update();

        // ========================================================
        // 0.5 [F2-HW-1] USB CDC Bootstrap & Reconnect Detection
        // ========================================================
        uint32_t now = millis();
        bool dtr_active = (bool)Serial;
        
        // 1. RX — Xử lý TRƯỚC TX để ưu tiên command/watchdog
        int rxBudget = 512; 
        bool received_this_cycle = false;
        while (Serial.available() && rxBudget > 0) {
            int b = Serial.read();
            if (b < 0) break;
            rxBudget--;
            received_this_cycle = true;

            SerialRxResult rx;
            if (g_parser.feed((uint8_t)b, rx, g_telemetry)) {
                handleRxFrame(rx, ctx);
            }
        }
        
        static uint32_t s_last_rx_ms = 0;
        if (received_this_cycle) {
            s_last_rx_ms = now;
        }

        // Theo dõi TX Drain
        size_t avail = Serial.availableForWrite();
        static size_t s_last_avail = 256;
        static uint32_t s_last_tx_drain_ms = 0;
        
        if (avail > s_last_avail) {
            // Có không gian trống mới -> host đã đọc data khỏi buffer
            s_last_tx_drain_ms = now;
        }
        
        // Đánh giá trạng thái kết nối
        bool rx_active = (now - s_last_rx_ms < 2000);
        bool tx_draining = (now - s_last_tx_drain_ms < 2000);
        bool hostNow = dtr_active || rx_active || tx_draining;

        if (hostNow && !g_lastHostConnected) {
            DBG.println("[SERIAL] USB host connected (DTR or Activity), resetting parser");
            g_parser.reset();
        } else if (!hostNow && g_lastHostConnected) {
            DBG.println("[SERIAL] USB host disconnected, flushing TX queue");
            Serial.flush();
        }
        g_lastHostConnected = hostNow;

        // Prevent bounce: update s_last_avail after any potential flush()
        s_last_avail = Serial.availableForWrite();

        // Bootstrap TX probe: nếu offline, gửi 1 frame mỗi 500ms để test drain
        bool is_probing = false;
        static uint32_t s_last_probe_ms = 0;
        if (!hostNow) {
            if (now - s_last_probe_ms >= 500) {
                s_last_probe_ms = now;
                is_probing = true;
            }
        }

        // ========================================================
        // 2. TX STATE @ 10Hz
        // [F2-3] Dùng g_txSeq chung, tăng sau mỗi frame được tạo.
        // [F2-6] Chỉ tăng tx_state khi serialSendFrame trả TX_OK.
        // ========================================================
        stateDivider++;
        if (stateDivider >= 5) {  // 20ms * 5 = 100ms = 10Hz
            stateDivider = 0;

            bool allow_tx_state = g_lastHostConnected || is_probing;
            if (!allow_tx_state) {
                // Host disconnected -> drop slot telemetry, không queue frame mới
                g_telemetry.tx_drop++;
            } else {
                // Copy odom và motion state dưới mutex, nhả mutex TRƯỚC khi format/write
                OdometryData_t odom;
                float gyro_z;
                uint32_t esp_ms = millis();

                xSemaphoreTake(ctx->stateMutex, portMAX_DELAY);
                odom = ctx->odom;
                gyro_z = ctx->motionState.gyro_z;
                xSemaphoreGive(ctx->stateMutex);

                // [F2-L2-4] Kiểm tra NaN/Inf trước khi format STATE
                if (!isfinite(odom.x) || !isfinite(odom.y) || !isfinite(odom.yaw) ||
                    !isfinite(odom.linear_velocity) || !isfinite(odom.angular_velocity) ||
                    !isfinite(gyro_z)) {
                    g_telemetry.tx_drop++;
                } else {
                    // Format STATE payload
                    int plen = snprintf(g_payloadBuf, sizeof(g_payloadBuf),
                        "STATE,2,%lu,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                        (unsigned long)g_txSeq,
                        (unsigned long)esp_ms,
                        odom.x, odom.y, odom.yaw,
                        odom.linear_velocity, odom.angular_velocity,
                        gyro_z);

                    if (plen > 0 && (size_t)plen < sizeof(g_payloadBuf)) {
                        SerialTxStatus st = serialSendFrame(g_txBuf, sizeof(g_txBuf),
                                                             g_payloadBuf, g_telemetry);
                        if (st == TX_OK) {
                            if (g_lastHostConnected) {
                                g_telemetry.tx_state++;
                            } else {
                                // [F2-HW-1] Probe frame queued but host not confirmed active.
                                // Tính là drop để tránh ảo giác telemetry.
                                g_telemetry.tx_drop++;
                            }
                        }
                        // [F2-L4-3] Chỉ tăng seq sau khi đã format và encode frame hoàn chỉnh
                        if (st != TX_ENCODE_FAIL) {
                            g_txSeq++;
                        }
                    } else {
                        g_telemetry.tx_drop++;
                    }
                }
            }
        }

        // ========================================================
        // 3. TX ENV @ 2Hz
        // [F2-3] Cùng g_txSeq chung.
        // [F2-6] Chỉ tăng tx_env khi TX_OK.
        // ========================================================
        envDivider++;
        if (envDivider >= 25) {  // 20ms * 25 = 500ms = 2Hz
            envDivider = 0;

            if (!g_lastHostConnected) {
                // [F2-L3-1] Host disconnected -> drop slot telemetry
                g_telemetry.tx_drop++;
            } else {
                // Copy sensor data dưới mutex
                bool dataValid;
                SensorPacket_t envData;

                xSemaphoreTake(ctx->stateMutex, portMAX_DELAY);
                dataValid = ctx->sensorDataValid;
                if (dataValid) {
                    envData = ctx->lastSensorData;
                }
                xSemaphoreGive(ctx->stateMutex);

                // [F2-L2-4] Check NaN/Inf and valid bounds for ENV
                if (dataValid) {
                    if (!isfinite(envData.gas_ppm) || !isfinite(envData.temperature) ||
                        !isfinite(envData.batt_voltage) || envData.fire_flags > 7) {
                        dataValid = false; // Dữ liệu lỗi → coi như offline/invalid
                    }
                }

                if (dataValid) {
                    int plen = snprintf(g_payloadBuf, sizeof(g_payloadBuf),
                        "ENV,2,%lu,%u,%.1f,%.1f,%.1f,1",
                        (unsigned long)g_txSeq,
                        (unsigned)envData.fire_flags,
                        envData.gas_ppm, envData.temperature,
                        envData.batt_voltage);

                    if (plen > 0 && (size_t)plen < sizeof(g_payloadBuf)) {
                        SerialTxStatus st = serialSendFrame(g_txBuf, sizeof(g_txBuf),
                                                             g_payloadBuf, g_telemetry);
                        if (st == TX_OK) {
                            g_telemetry.tx_env++;
                        }
                        // [F2-L4-3] Tăng seq
                        if (st != TX_ENCODE_FAIL) {
                            g_txSeq++;
                        }
                    } else {
                        g_telemetry.tx_drop++;
                    }
                } else {
                    // WROOM offline: valid=0
                    int plen = snprintf(g_payloadBuf, sizeof(g_payloadBuf),
                        "ENV,2,%lu,0,0.0,0.0,0.0,0",
                        (unsigned long)g_txSeq);

                    if (plen > 0 && (size_t)plen < sizeof(g_payloadBuf)) {
                        SerialTxStatus st = serialSendFrame(g_txBuf, sizeof(g_txBuf),
                                                             g_payloadBuf, g_telemetry);
                        if (st == TX_OK) {
                            g_telemetry.tx_env++;
                        }
                        // [F2-L4-3] Tăng seq
                        if (st != TX_ENCODE_FAIL) {
                            g_txSeq++;
                        }
                    } else {
                        g_telemetry.tx_drop++;
                    }
                }
            }
        }

        // ========================================================
        // 4. TELEMETRY LOG mỗi 10 giây (qua DBG/Serial0)
        // ========================================================
        now = millis();
        if ((now - lastTelemetryMs) >= 10000) {
            lastTelemetryMs = now;

            uint32_t cmd_age_ms = 0;
            if (g_telemetry.last_valid_cmd_ms > 0) {
                cmd_age_ms = now - g_telemetry.last_valid_cmd_ms;
            }

            DBG.println("--- [SERIAL TELEMETRY] ---");
            DBG.printf("  RX: valid=%lu crc_fail=%lu parse_fail=%lu overflow=%lu\n",
                g_telemetry.rx_valid, g_telemetry.rx_crc_fail,
                g_telemetry.rx_parse_fail, g_telemetry.rx_overflow);
            DBG.printf("  RX seq: gap=%lu dup=%lu\n",
                g_telemetry.rx_seq_gap, g_telemetry.rx_seq_dup);
            DBG.printf("  TX: state=%lu env=%lu drop=%lu partial=%lu\n",
                g_telemetry.tx_state, g_telemetry.tx_env,
                g_telemetry.tx_drop, g_telemetry.tx_partial);
            DBG.printf("  CMD: last=%lums ago=%lums\n",
                g_telemetry.last_valid_cmd_ms, cmd_age_ms);
            DBG.printf("  heap: free=%u min=%u\n",
                ESP.getFreeHeap(), ESP.getMinFreeHeap());
            DBG.printf("  stack_hwm: Task_Serial=%u words\n",
                uxTaskGetStackHighWaterMark(nullptr));
            DBG.println("--------------------------");
        }
    }
}
