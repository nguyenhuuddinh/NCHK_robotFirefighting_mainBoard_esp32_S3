/**
 * @file TaskSerialComm.cpp
 * @brief FreeRTOS Task cho Raw Serial V2 Communication (Core 0)
 *
 * Thay thế Task_MicroROS trong runtime chính.
 * Chạy trên Core 0, Priority 3, chu kỳ 20ms.
 *
 * TX (ESP32 -> Pi):
 *   STATE @ 10Hz (mỗi 100ms): odom + motion data
 *   ENV   @ 2Hz  (mỗi 500ms): legacy WROOM slot; disabled in S3-only build
 *   IR    @ 5Hz  (mỗi 200ms): ADS1115 raw, read-only
 *   ACT   @ 2Hz  (best effort): commanded actuator state, not physical feedback
 *
 * RX (Pi -> ESP32):
 *   CMD:  cập nhật cmdVel + notify watchdog (duplicate bị bỏ qua)
 *   FIRE: legacy servo-pan queue; GPIO38 not actuated in this phase
 *   PUMP: GPIO47 pump PWM through a bounded, host/robot-watchdog gate
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
#include "PumpOutput.h"
#include "PumpSafety.h"
#include "ServoConfig.h"
#if SERVO_SWEEP_ENABLED
#include "ServoOutput.h"
#include "ServoSweep.h"
#endif
#include <Arduino.h>
#include <math.h>
#include <soc/usb_wrap_struct.h>
#include <tusb.h>

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



// [F2-3] TX sequence counter CHUNG cho STATE, ENV, IR, ACT
// Mỗi hướng truyền có MỘT bộ đếm. Tăng sau mỗi frame được tạo.
static uint32_t g_txSeq = 0;

// Trạng thái actuator tĩnh (giống pattern trong MicroRosComm callbacks)
static uint8_t s_servo_pan = 90;
static uint8_t s_pump_on   = 0;
static uint8_t s_buzzer_on = 0;
static PumpSafety s_pump_safety;
#if SERVO_SWEEP_ENABLED
static ServoSweep s_servo_sweep(SERVO_SWEEP_START_DEG, SERVO_SWEEP_TURN_DEG,
                                SERVO_SWEEP_END_DEG, SERVO_SWEEP_STEP_PERIOD_MS);
static bool s_servo_fault = false;
static uint32_t s_home_completion_count = 0;
// Updated only from validated CMD frames on this same Core-0 task.
static bool s_drive_command_zero = false;
#endif

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
#if SERVO_SWEEP_ENABLED
        s_drive_command_zero = isfinite(rx.f1) && isfinite(rx.f2) &&
                               fabsf(rx.f1) < 0.001f && fabsf(rx.f2) < 0.001f;
#endif
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
        // Direct ESP32-S3 pump owner. The old WROOM queue is not a live output.
        s_pump_on = rx.pump;
        s_pump_safety.request(s_pump_on != 0, millis());
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
    // Phase IR two ticks (40ms) after STATE, allowing its USB frame to drain.
    uint8_t irDivider = 8;
    // Offset ACT away from the 100 ms STATE slots. Lower priority than IR.
    uint8_t actDivider = 18;

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
        const uint32_t pre_rx_ms = millis();

        // 1. Khai báo các trạng thái liveness/probe tĩnh
        static TxLivenessStateMachine sm;
        static TxScheduler txSched;
        static UsbRecoveryController usbRecovery;
        static uint32_t s_last_probe_ms = 0;
        static uint32_t s_last_disconnect_epoch = 0;
        static bool     s_discarding = false;

        bool fast_disconnect = false;

        if (usbRecovery.poll(pre_rx_ms) == UsbRecoveryAction::ATTACH) {
            DBG.println("[SERIAL] USB CDC recovery reconnect");
            USB_WRAP.otg_conf.pad_enable = true;
            if (!tud_connect()) {
                DBG.println("[SERIAL] USB CDC reconnect request failed");
            }
        }

        // [F2-L17-1] Helper cleanup idempotent
        auto cleanup_session = [&]() {
            s_last_disconnect_epoch = s_disconnect_epoch;
            fast_disconnect = true;
            g_parser.reset();
            sm.reset();
            g_lastHostConnected = false;
            s_discarding = true;
        };

        auto cleanup_disconnected_session = [&]() {
            usbRecovery.observeHostDisconnect();
            cleanup_session();
        };

        // (a) Checkpoint trước read
        if (s_disconnect_epoch != s_last_disconnect_epoch) {
            cleanup_disconnected_session();
        }

        // ========================================================
        // 1. Đọc Serial (RX) - Bounded 512 bytes
        // BẮT BUỘC NẰM TRƯỚC TÍNH TOÁN hostNow VÀ TX PUMP
        // ========================================================
        int rxBudget = 512;
        bool valid_rx_activity = false;

        while (Serial.available() > 0 && rxBudget > 0) {
            if (s_disconnect_epoch != s_last_disconnect_epoch) {
                cleanup_disconnected_session();
            }

            int b = Serial.read();
            if (b < 0) break;

            g_telemetry.raw_rx_bytes++;
            rxBudget--;

            // (b) Checkpoint ngay sau Serial.read và trước g_parser.feed
            if (s_disconnect_epoch != s_last_disconnect_epoch) {
                cleanup_disconnected_session();
            }

            if (s_discarding) {
                // Chỉ đọc bỏ, tuyệt đối không feed parser
                continue;
            }

            SerialRxResult rx;
            if (g_parser.feed((uint8_t)b, rx, g_telemetry)) {
                // (c) Checkpoint sau feed nhưng trước handleRxFrame
                if (s_disconnect_epoch != s_last_disconnect_epoch) {
                    cleanup_disconnected_session();
                    continue;
                }
                valid_rx_activity = true;
                if (usbRecovery.observeValidRx()) {
                    DBG.printf(
                        "[SERIAL] USB recovery armed by valid RX type=%d\n",
                        (int)rx.type);
                }
                handleRxFrame(rx, ctx);
            }
        }

        // (d) Checkpoint sau RX loop trước clear-discard/hostNow/TX
        if (s_disconnect_epoch != s_last_disconnect_epoch) {
            cleanup_disconnected_session();
        }

        // Khi queue rỗng trong quá trình discard, clear discard_mode và reset parser lần cuối
        if (s_discarding && Serial.available() == 0) {
            s_discarding = false;
            g_parser.reset();
        }

        // RX_PUMP timestamps its lease with millis() inside handleRxFrame.
        // Sample again after RX so PumpSafety::tick never sees a time earlier
        // than the latest request and falsely latches LEASE_EXPIRED.
        uint32_t now = millis();

        // ========================================================
        // 2. Tính/Recompute hostNow TỪ LIVENESS STATE MACHINE
        // ========================================================
        if (fast_disconnect) {
            sm.reset();
            fast_disconnect = false;
        } else {
            sm.update(
                s_dtr_active, s_usb_tx_event_cnt, valid_rx_activity);
        }

        // DTR drop chỉ đóng session logic. Chỉ TX stall khi host còn active
        // mới được phép thực hiện physical USB detach/attach một lần.
        if (sm.recovery_pending) {
            const SessionRecoveryReason recoveryReason = sm.recovery_reason;
            if (recoveryReason == SessionRecoveryReason::HOST_DISCONNECTED) {
                if (g_lastHostConnected) {
                    DBG.println("[SERIAL] USB host disconnected -> Offline");
                }
                usbRecovery.observeHostDisconnect();
            } else if (recoveryReason == SessionRecoveryReason::TX_STALLED) {
                DBG.println("[SERIAL] USB CDC TX stalled -> Recovery");
            }
            cleanup_session(); // resets sm internally, clears recovery_pending

            // Physical recovery is reserved for a confirmed TX stall. A normal
            // host close must leave the USB device attached and passively wait
            // for the next DTR/RX session instead of creating an xHCI storm.
            if (recoveryReason == SessionRecoveryReason::TX_STALLED &&
                usbRecovery.request(now) == UsbRecoveryAction::DETACH) {
                DBG.println("[SERIAL] USB CDC stalled -> soft reconnect");
                tud_cdc_n_write_clear(0);
                tud_cdc_n_read_flush(0);
                // Soft detach does not guarantee that Arduino's asynchronous
                // CDC event loop runs before the next task iteration. Invalidate
                // the cached line state synchronously so stale DTR cannot arm
                // probes against the detached endpoint.
                s_dtr_active = false;
                s_disconnect_epoch++;
                if (!tud_disconnect()) {
                    DBG.println("[SERIAL] USB CDC disconnect request failed");
                }
                // ESP32-S3's DWC soft-disconnect bit is not reliably
                // observable by Linux after repeated recoveries. Gate the
                // internal PHY pad as well so every recovery produces a real
                // D+/D- disconnect without rebooting the controller.
                USB_WRAP.otg_conf.pad_enable = false;
            }
        }

        bool hostNow = (sm.state == SessionState::ONLINE);

        // Hard lease + continuous-run cap are independent of the ROS client.
        // Even if the Pi repeats stale PUMP=1, the pump cannot run indefinitely.
        // The motor command watchdog is an actuator interlock as well: a
        // disconnected or EMERGENCY/IDLE robot cannot keep spraying.
        const bool actuator_link_ok = hostNow && robotMaster.isMotionAllowed();
        const bool pump_request_allowed = s_pump_safety.tick(now, actuator_link_ok);
#if SERVO_SWEEP_ENABLED
        const bool safe_to_home = actuator_link_ok && s_drive_command_zero;
        if (s_servo_sweep.returningHome() && !safe_to_home) {
            s_servo_sweep.abort();
            servoOutput.stop();
            DBG.println("[ERROR][SERVO] Home interrupted; pump locked until reboot.");
        }
        // Return only after the full sweep, intentional OFF or the bounded
        // run cap. Lease expiry and link/watchdog loss remain fail-closed.
        if (s_servo_sweep.active() && !pump_request_allowed) {
            const bool normal_stop = !s_pump_on ||
                s_pump_safety.stopReason() == PumpSafety::StopReason::MAX_RUN;
            if (normal_stop && safe_to_home && s_servo_sweep.beginReturn(now)) {
                DBG.println("[SERVO] Pump OFF; returning nozzle to 90 deg.");
            } else {
                s_servo_sweep.abort();
                servoOutput.stop();
                DBG.println("[ERROR][SERVO] Sweep interrupted; pump locked until reboot.");
            }
        }
        // Re-arm requires physical settling at 90 and an observed PUMP=0.
        // A held PUMP=1 cannot trigger another cycle after the 5 s cap.
        if (s_servo_sweep.returningHome() && !s_pump_on && safe_to_home &&
            s_servo_sweep.rearm(now)) {
            s_home_completion_count++;
            DBG.println("[SERVO] Nozzle at 90 deg; new OFF->ON cycle allowed.");
        }
        const bool pump_allowed = pump_request_allowed &&
                                  !s_servo_sweep.consumed() && !s_servo_fault;
#else
        const bool pump_allowed = pump_request_allowed;
#endif
        pumpOutput.setEnabled(pump_allowed);
#if SERVO_SWEEP_ENABLED
        // Use gated pump state; while returning, update() keeps the servo
        // energized at 90 for a short settle despite the pump being OFF.
        const int nozzle_angle = s_servo_sweep.update(
            pump_allowed && pumpOutput.ready(), now);
        if (nozzle_angle < 0) {
            if (servoOutput.active()) servoOutput.stop();
            if (pump_allowed) {
                s_servo_fault = true;
                s_servo_sweep.abort();
                pumpOutput.setEnabled(false);
                DBG.println("[ERROR][SERVO] Sweep unavailable; pump OFF until reboot.");
            }
        } else if (!servoOutput.writeAngle(nozzle_angle)) {
            s_servo_fault = true;
            s_servo_sweep.abort();
            pumpOutput.setEnabled(false);
            servoOutput.stop();
            DBG.println("[ERROR][SERVO] PWM write failed; pump and servo OFF until reboot.");
        }
#endif

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
                bool imu_valid;
                uint32_t esp_ms = millis();

                xSemaphoreTake(ctx->stateMutex, portMAX_DELAY);
                odom = ctx->odom;
                gyro_z = ctx->motionState.gyro_z;
                imu_valid = ctx->imuValid;
                xSemaphoreGive(ctx->stateMutex);

                // STATE carries both odom and IMU with no validity bit. Drop
                // the whole frame on an invalid gyro; Pi must not publish a
                // fabricated /imu/data value as a fresh measurement.
                if (!imu_valid || !isfinite(odom.x) || !isfinite(odom.y) || !isfinite(odom.yaw) ||
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

        // IR is deliberately queued after STATE/ENV. It may drop when the
        // two-slot USB TX ring is busy; navigation telemetry keeps priority.
        // Only copy the completed ADS1115 snapshot; never touch I2C here.
        if (++irDivider >= 10) {
            irDivider = 0;
            if (sm.state != SessionState::ONLINE || sm.tx_count != 0 ||
                Serial.availableForWrite() < 64) {
                g_telemetry.tx_drop++;
            } else {
                uint16_t raw[3];
                uint32_t sample_ms;
                bool valid;
                xSemaphoreTake(ctx->stateMutex, portMAX_DELAY);
                raw[0] = ctx->irRaw[0];
                raw[1] = ctx->irRaw[1];
                raw[2] = ctx->irRaw[2];
                sample_ms = ctx->irSampleMs;
                valid = ctx->irValid;
                xSemaphoreGive(ctx->stateMutex);

                const uint32_t age_ms = millis() - sample_ms;
                valid = valid && sample_ms != 0 && age_ms <= 300 &&
                        raw[0] <= 32767 && raw[1] <= 32767 && raw[2] <= 32767;
                if (!valid) {
                    raw[0] = raw[1] = raw[2] = 0;
                }
                const int plen = snprintf(g_payloadBuf, sizeof(g_payloadBuf),
                    "IR,2,%lu,%lu,%u,%u,%u,%u",
                    (unsigned long)g_txSeq, (unsigned long)age_ms,
                    (unsigned)raw[0], (unsigned)raw[1], (unsigned)raw[2],
                    (unsigned)(valid ? 1 : 0));
                if (plen > 0 && (size_t)plen < sizeof(g_payloadBuf)) {
                    if (sm.enqueue(g_payloadBuf, 4, now + 100, g_telemetry)) {
                        g_txSeq++;
                    }
                } else {
                    g_telemetry.tx_drop++;
                }
            }
        }

#if SERVO_SWEEP_ENABLED
        // Read-only commanded-state snapshot. Never read sensors or block
        // STATE/IR transmission for this optional 2 Hz frame.
        if (++actDivider >= 25) {
            actDivider = 0;
            if (sm.state == SessionState::ONLINE && sm.tx_count == 0 &&
                Serial.availableForWrite() >= 64) {
                const bool servo_pwm = servoOutput.active();
                const uint8_t phase = (s_servo_fault ||
                    (s_servo_sweep.consumed() &&
                     !s_servo_sweep.returningHome())) ? 3 :
                    s_servo_sweep.returningHome() ? 2 :
                    s_servo_sweep.active() ? 1 : 0;
                const unsigned angle = servo_pwm && nozzle_angle >= 0 ?
                    (unsigned)nozzle_angle : 255;
                const int plen = snprintf(g_payloadBuf, sizeof(g_payloadBuf),
                    "ACT,2,%lu,%lu,%u,%u,%u,%u,%lu,%u",
                    (unsigned long)g_txSeq, (unsigned long)now,
                    (unsigned)pumpOutput.commandedOn(), (unsigned)phase,
                    angle, (unsigned)servo_pwm,
                    (unsigned long)s_home_completion_count,
                    (unsigned)pumpOutput.ready());
                if (plen > 0 && (size_t)plen < sizeof(g_payloadBuf)) {
                    if (sm.enqueue(g_payloadBuf, 5, now + 150, g_telemetry)) {
                        g_txSeq++;
                    }
                } else {
                    g_telemetry.tx_drop++;
                }
            } else {
                g_telemetry.tx_drop++;
            }
        }
#endif

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
