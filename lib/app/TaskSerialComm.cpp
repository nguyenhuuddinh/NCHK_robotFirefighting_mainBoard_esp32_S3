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

// TX Ring Queue moved to TxLivenessStateMachine in SerialComm.h

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
// DTR active chỉ mở state PROBING.
// Trạng thái ONLINE chỉ đạt được sau khi có TX-complete event mới hơn
// pre-write probe baseline.
// DTR fall, disconnect, hoặc three partials sẽ đóng phiên.
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
        static TxLivenessStateMachine sm;
        static TxScheduler txSched;
        static uint32_t s_last_probe_ms = 0;
        static uint32_t s_last_disconnect_epoch = 0;
        static bool     s_discarding = false;

        bool fast_disconnect = false;

        // [F2-L17-1] Helper cleanup idempotent
        auto cleanup_session = [&]() {
            s_last_disconnect_epoch = s_disconnect_epoch;
            fast_disconnect = true;
            g_parser.reset();
            sm.reset();
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
        // 2. Tính/Recompute hostNow TỪ LIVENESS STATE MACHINE
        // ========================================================
        if (fast_disconnect) {
            sm.reset();
            fast_disconnect = false;
        } else {
            sm.update(s_dtr_active, s_usb_tx_event_cnt);
        }

        // [F2-L26-1] Nếu state machine yêu cầu recovery (DTR drop hoặc >= 3 TX stall),
        // phải cleanup ngay để xóa stale queue/parser trước khi caller cho nó quay lại PROBING
        if (sm.recovery_pending) {
            if (g_lastHostConnected) {
                DBG.println("[SERIAL] USB host disconnected or stalled -> Recovery");
            }
            cleanup_session(); // resets sm internally, clears recovery_pending
        }

        bool hostNow = (sm.state == SessionState::ONLINE);

        if (hostNow && !g_lastHostConnected) {
            DBG.println("[SERIAL] USB host connected (Activity confirmed)");
        }
        g_lastHostConnected = hostNow;

        // ========================================================
        // [F2-L27-1] Probe đã chốt TX-event baseline trước khi vào pump.
        // ========================================================
        sm.pump_tx(now, g_telemetry);

        // ========================================================
        // 2. TX STATE @ 10Hz
        // [F2-L7-2] Counter semantics:
        //   tx_state = STATE frame queued vào ring buffer.
        //   tx_probe = probe frame queued (bootstrap, chưa xác nhận host).
        //   tx_drop  = slot/frame bị bỏ (offline, NaN, buffer full).
        //   Host delivery chỉ xác nhận bởi drain activity.
        //   Protocol V2 không có per-frame ACK.
        // ========================================================
        txSched.tick();
        if (txSched.state_due) {

            bool allow_tx_state = (sm.state == SessionState::ONLINE);
            bool need_probe = (sm.state == SessionState::PROBING && sm.tx_count == 0 && (now - s_last_probe_ms >= 500));

            if (!allow_tx_state && !need_probe) {
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
                        if (need_probe) {
                            if (sm.enqueue_probe(
                                    g_payloadBuf, now + 100,
                                    &s_usb_tx_event_cnt, g_telemetry)) {
                                s_last_probe_ms = now;
                                g_txSeq++;
                            }
                        } else {
                            if (sm.enqueue(g_payloadBuf, 1, now + 100, g_telemetry)) {
                                g_txSeq++;
                            }
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
        if (txSched.env_due) {

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
                        if (sm.enqueue(g_payloadBuf, 2, now + 500, g_telemetry)) {
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
                        if (sm.enqueue(g_payloadBuf, 2, now + 500, g_telemetry)) {
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

            // Log TinyUSB CDC events [F2-L26-5]
            DBG.printf("  host: up=%d dtr=%d usb_evt=%lu tx_evt=%lu state=%d part=%lu arm=%d base=%lu\n",
                (int)g_lastHostConnected, (int)s_dtr_active, s_usb_event_cnt, s_usb_tx_event_cnt,
                (int)sm.state, sm.consecutive_partials, (int)sm.probe_armed, sm.probe_armed_tx_event_cnt);

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
