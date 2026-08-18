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

// [F2-L14-1] TinyUSB CDC Cooperative Segmented TX Queue (Ring FIFO)
struct TxSlot {
    char data[SERIAL_BUFFER_SIZE]; // [F2-L14-2]
    uint16_t len;
    uint16_t offset;
    uint8_t type; // 1=STATE, 2=ENV, 3=PROBE
    uint32_t deadline_ms;
};

static TxSlot s_txQueue[2];
static volatile uint8_t s_txHead = 0;
static volatile uint8_t s_txTail = 0;
static volatile uint8_t s_txCount = 0;

extern RobotMaster robotMaster;

// ============================================================
// STATIC INSTANCES (tồn tại suốt đời sống chương trình)
// ============================================================
static SerialParser   g_parser;
static SerialCommTelemetry g_telemetry;

// [F2-L13-1] TinyUSB CDC Event Callbacks
static volatile uint32_t s_usb_event_cnt = 0;
static volatile uint32_t s_usb_tx_event_cnt = 0;
static volatile bool s_dtr_active = false;
static volatile uint32_t s_disconnect_epoch = 0;

static void usb_event_cb(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == ARDUINO_USB_CDC_EVENTS) {
        s_usb_event_cnt++;
        if (event_id == ARDUINO_USB_CDC_LINE_STATE_EVENT) {
            arduino_usb_cdc_event_data_t* data = (arduino_usb_cdc_event_data_t*)event_data;
            bool new_dtr = data->line_state.dtr;
            // [F2-L15-2] DTR falling edge -> session close
            if (s_dtr_active && !new_dtr) {
                s_disconnect_epoch++;
            }
            s_dtr_active = new_dtr;
        } else if (event_id == ARDUINO_USB_CDC_TX_EVENT) {
            s_usb_tx_event_cnt++;
        } else if (event_id == ARDUINO_USB_CDC_DISCONNECTED_EVENT) {
            s_dtr_active = false;
            s_disconnect_epoch++; // [F2-L14-5]
        }
    }
}

static bool enqueueFrame(const char* payload, uint8_t type, uint32_t deadline_ms) {
    g_telemetry.tx_generated++;
    // [F2-L14-1] Queue full -> drop
    if (s_txCount >= 2) {
        g_telemetry.tx_drop++;
        return false;
    }

    // Encode into tail
    size_t len = serialEncodeFrame(s_txQueue[s_txTail].data, SERIAL_BUFFER_SIZE, payload);
    if (len > 0) {
        s_txQueue[s_txTail].len = len;
        s_txQueue[s_txTail].offset = 0;
        s_txQueue[s_txTail].type = type;
        s_txQueue[s_txTail].deadline_ms = deadline_ms;

        s_txTail = (s_txTail + 1) % 2;
        s_txCount++;
        g_telemetry.tx_queued++;
        return true;
    }
    g_telemetry.tx_drop++;
    return false;
}

// [F2-3] TX sequence counter CHUNG cho cả STATE và ENV
// Mỗi hướng truyền có MỘT bộ đếm. Tăng sau mỗi frame được tạo.
static uint32_t g_txSeq = 0;

// Trạng thái actuator tĩnh (giống pattern trong MicroRosComm callbacks)
static uint8_t s_servo_pan = 90;
static uint8_t s_pump_on   = 0;
static uint8_t s_buzzer_on = 0;

// TX buffer tĩnh — không malloc
static char g_payloadBuf[SERIAL_BUFFER_SIZE];

// [F2-L8-1] USB CDC connection detection
// USBCDC operator bool() = isPlugged() && connected
// Sau khi host process đóng port mà cáp vẫn cắm, operator bool có thể
// vẫn true vô thời hạn (SOF vẫn chạy). Do đó KHÔNG dùng nó như liveness.
// Host liveness chỉ dựa trên RX activity hoặc probe drain confirmation.
// operator bool chỉ dùng như bootstrap hint có thời hạn.
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

    // ========================================================
    // [F2-L13-1] Đăng ký TinyUSB CDC event callback 1 lần
    // ========================================================
    static bool s_event_registered = false;
    if (!s_event_registered) {
        Serial.onEvent(usb_event_cb);
        s_event_registered = true;
    }

    for (;;) {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(TASK_SERIALCOMM_PERIOD_MS));

        // ========================================================
        // 0. CẬP NHẬT STATE MACHINE (Watchdog check)
        //    Giống TaskMicroROS: gọi robotMaster.update() đều đặn
        // ========================================================
        robotMaster.update();

        // ========================================================
        // 0.5 [F2-L9-1] USB CDC Bootstrap & Reconnect Detection
        // ========================================================
        uint32_t now = millis();

        // 1. Khai báo các trạng thái liveness/probe tĩnh
        static uint32_t s_last_rx_ms = 0;
        static uint32_t s_last_tx_activity_ms = 0;
        static uint32_t s_last_tx_event_cnt = 0;
        static bool     s_probe_pending        = false;
        static bool     s_probe_outstanding    = false;
        static uint32_t s_last_probe_ms        = 0;
        static uint32_t s_last_disconnect_epoch = 0;
        static bool     s_discarding = false;

        // Cập nhật drain activity
        uint32_t current_tx_event_cnt = s_usb_tx_event_cnt;
        if (current_tx_event_cnt != s_last_tx_event_cnt) {
            s_last_tx_event_cnt = current_tx_event_cnt;
            s_last_tx_activity_ms = now;
            s_probe_outstanding = false;
        }

        bool fast_disconnect = false;

        // [F2-L17-1] Helper cleanup idempotent
        auto cleanup_session = [&]() {
            s_last_disconnect_epoch = s_disconnect_epoch;
            fast_disconnect = true;
            g_parser.reset();
            s_probe_pending = false;
            s_probe_outstanding = false;
            s_txHead = 0; s_txTail = 0; s_txCount = 0;
            s_last_rx_ms = 0;
            s_last_tx_activity_ms = 0;
            g_lastHostConnected = false;
            s_discarding = true;
        };

        // (a) Checkpoint trước read
        if (s_disconnect_epoch != s_last_disconnect_epoch) {
            cleanup_session();
        }

        // ========================================================
        // 1. Đọc Serial (RX) - Bounded 512 bytes
        // BẮT BUỘC NẰM TRƯỚC TÍNH TOÁN hostNow VÀ TX PUMP
        // ========================================================
        int rxBudget = 512;

        while (Serial.available() > 0 && rxBudget > 0) {
            if (s_disconnect_epoch != s_last_disconnect_epoch) {
                cleanup_session();
            }

            int b = Serial.read();
            if (b < 0) break;

            g_telemetry.raw_rx_bytes++;
            rxBudget--;

            // (b) Checkpoint ngay sau Serial.read và trước g_parser.feed
            if (s_disconnect_epoch != s_last_disconnect_epoch) {
                cleanup_session();
            }

            if (s_discarding) {
                // Chỉ đọc bỏ, tuyệt đối không feed parser
                continue;
            }

            s_last_rx_ms = now;

            SerialRxResult rx;
            if (g_parser.feed((uint8_t)b, rx, g_telemetry)) {
                // (c) Checkpoint sau feed nhưng trước handleRxFrame
                if (s_disconnect_epoch != s_last_disconnect_epoch) {
                    cleanup_session();
                    continue;
                }
                handleRxFrame(rx, ctx);
            }
        }

        // (d) Checkpoint sau RX loop trước clear-discard/hostNow/TX
        if (s_disconnect_epoch != s_last_disconnect_epoch) {
            cleanup_session();
        }

        // Khi queue rỗng trong quá trình discard, clear discard_mode và reset parser lần cuối
        if (s_discarding && Serial.available() == 0) {
            s_discarding = false;
            g_parser.reset();
        }

        // ========================================================
        // 2. Tính/Recompute hostNow TỪ RX/TX ACTIVITY
        // ========================================================
        bool rx_active    = (s_last_rx_ms > 0 && (now - s_last_rx_ms < 2000));
        bool drain_active = (s_last_tx_activity_ms > 0 && (now - s_last_tx_activity_ms < 2000));
        bool hostNow = (rx_active || drain_active) && !fast_disconnect;

        // Nếu disconnect
        if (fast_disconnect || (!hostNow && g_lastHostConnected)) {
            DBG.println("[SERIAL] USB host disconnected (event/timeout)");
            g_parser.reset();
            s_probe_pending = false;
            s_probe_outstanding = false;
            s_txHead = 0; s_txTail = 0; s_txCount = 0;
            s_last_rx_ms = 0;
            s_last_tx_activity_ms = 0;
            hostNow = false;
            s_discarding = true; // Bắt đầu vào discard mode cho cycle sau nếu còn byte
        } else if (hostNow && !g_lastHostConnected) {
            DBG.println("[SERIAL] USB host connected (Activity confirmed)");
            // [F2-L16-1] KHÔNG reset parser ở đây để giữ last_rx_seq của frame đầu tiên
        }
        g_lastHostConnected = hostNow;

        // --- Bootstrap hint ---
        {
            static bool s_last_bootstrap = false;
            bool current_bootstrap = s_dtr_active || (Serial.availableForWrite() > 0);
            if (current_bootstrap && !s_last_bootstrap && !fast_disconnect && !s_discarding) {
                if (!s_probe_outstanding) {
                    s_probe_pending = true;
                }
            }
            s_last_bootstrap = current_bootstrap;
        }

        // ========================================================
        // [F2-L14-1] Cooperative Segmented TX Loop (Ring FIFO)
        // Mỗi chu kỳ đẩy tối đa 64 byte để không block
        // ========================================================
        if ((hostNow || s_probe_pending || s_probe_outstanding) && s_txCount > 0) {
            TxSlot* head = &s_txQueue[s_txHead];
            const bool failed_probe = (head->type == 3);

            // [F2-L14-5] Wrap-safe deadline check
            if ((int32_t)(now - head->deadline_ms) >= 0) {
                g_telemetry.tx_partial++;
                s_txCount--;
                s_txHead = (s_txHead + 1) % 2;
                if (failed_probe) {
                    s_probe_outstanding = false;
                    s_probe_pending = false;
                }
            } else {
                size_t remaining = head->len - head->offset;
                size_t avail = Serial.availableForWrite();
                size_t chunk = remaining;
                if (chunk > 64) chunk = 64;
                if (chunk > avail) chunk = avail;

                if (chunk > 0) {
                    size_t written = Serial.write((const uint8_t*)&head->data[head->offset], chunk);
                    if (written == chunk) {
                        head->offset += written;
                        if (head->offset >= head->len) {
                            g_telemetry.tx_completed++;
                            s_txCount--;
                            s_txHead = (s_txHead + 1) % 2;
                        }
                    } else {
                        // [F2-L14-4] written == 0 hoặc written < chunk -> Abort ngay
                        g_telemetry.tx_partial++;
                        s_txCount--;
                        s_txHead = (s_txHead + 1) % 2;
                        if (failed_probe) {
                            s_probe_outstanding = false;
                            s_probe_pending = false;
                        }
                    }
                }
            }
        }

        // Probe scheduling
        if (hostNow) {
            s_probe_pending = false;
            s_probe_outstanding = false;
        } else {
            if (!s_probe_outstanding && (now - s_last_probe_ms >= 500)) {
                s_probe_pending = true;
            }
        }

        // ========================================================
        // 2. TX STATE @ 10Hz
        // [F2-L7-2] Counter semantics:
        //   tx_state = STATE frame queued vào ring buffer.
        //   tx_probe = probe frame queued (bootstrap, chưa xác nhận host).
        //   tx_drop  = slot/frame bị bỏ (offline, NaN, buffer full).
        //   Host delivery chỉ xác nhận bởi drain activity hoặc RX.
        //   Protocol V2 không có per-frame ACK.
        // ========================================================
        stateDivider++;
        if (stateDivider >= 5) {  // 20ms * 5 = 100ms = 10Hz
            stateDivider = 0;

            // Probe được tiêu thụ tại STATE TX slot
            bool consuming_probe = false;
            if (!g_lastHostConnected && s_probe_pending) {
                consuming_probe = true;
                s_probe_pending = false;
            }
            bool allow_tx_state = g_lastHostConnected || consuming_probe;

            if (!allow_tx_state) {
                g_telemetry.tx_drop++;
            } else {
                OdometryData_t odom;
                float gyro_z;
                uint32_t esp_ms = millis();

                xSemaphoreTake(ctx->stateMutex, portMAX_DELAY);
                odom = ctx->odom;
                gyro_z = ctx->motionState.gyro_z;
                xSemaphoreGive(ctx->stateMutex);

                if (!isfinite(odom.x) || !isfinite(odom.y) || !isfinite(odom.yaw) ||
                    !isfinite(odom.linear_velocity) || !isfinite(odom.angular_velocity) ||
                    !isfinite(gyro_z)) {
                    g_telemetry.tx_drop++;
                } else {
                    int plen = snprintf(g_payloadBuf, sizeof(g_payloadBuf),
                        "STATE,2,%lu,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                        (unsigned long)g_txSeq,
                        (unsigned long)esp_ms,
                        odom.x, odom.y, odom.yaw,
                        odom.linear_velocity, odom.angular_velocity,
                        gyro_z);

                    if (plen > 0 && (size_t)plen < sizeof(g_payloadBuf)) {
                        // Queue với deadline 100ms
                        if (enqueueFrame(g_payloadBuf, consuming_probe ? 3 : 1, now + 100)) {
                            if (consuming_probe) {
                                s_probe_outstanding = true;
                                s_last_probe_ms = now;
                            }
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
                        // ENV deadline 500ms
                        if (enqueueFrame(g_payloadBuf, 2, now + 500)) {
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
                        if (enqueueFrame(g_payloadBuf, 2, now + 500)) {
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

            // [F2-L13-1] Cập nhật diagnostics log cho TinyUSB CDC
            g_telemetry.usb_events = s_usb_event_cnt;
            DBG.println("--- [SERIAL TELEMETRY] ---");
            DBG.printf("  RX: raw_bytes=%lu valid=%lu crc_fail=%lu parse_fail=%lu overflow=%lu\n",
                g_telemetry.raw_rx_bytes,
                g_telemetry.rx_valid, g_telemetry.rx_crc_fail,
                g_telemetry.rx_parse_fail, g_telemetry.rx_overflow);
            DBG.printf("  RX seq: gap=%lu dup=%lu\n",
                g_telemetry.rx_seq_gap, g_telemetry.rx_seq_dup);
            DBG.printf("  TX: gen=%lu queued=%lu comp=%lu drop=%lu part=%lu\n",
                g_telemetry.tx_generated, g_telemetry.tx_queued,
                g_telemetry.tx_completed, g_telemetry.tx_drop,
                g_telemetry.tx_partial);

            // Log TinyUSB CDC events
            DBG.printf("  host: up=%d dtr=%d usb_evt=%lu probe_pend=%d probe_out=%d\n",
                (int)g_lastHostConnected, (int)s_dtr_active, s_usb_event_cnt,
                (int)s_probe_pending, (int)s_probe_outstanding);

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
