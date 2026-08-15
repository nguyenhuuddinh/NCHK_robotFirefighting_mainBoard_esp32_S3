/**
 * @file MicroRosComm.cpp
 * @brief Trien khai module giao tiep micro-ROS
 *
 * Vong doi:
 *   1. init() — Setup Serial transport, ping Agent
 *   2. spinOnce() — State machine: WAITING → CONNECTED ↔ DISCONNECTED
 *   3. destroyEntities_() — Cleanup truoc khi reconnect
 *
 * Transport: USB Serial (Serial = USB CDC) → Raspberry Pi micro-ROS Agent
 * Debug log: DBG (Serial0 = UART0/CH340) → Laptop Serial Monitor
 * Node name: "motion_slave"
 * Namespace: "" (root)
 *
 * Publish: /odom, /imu/data, /env_status (Best Effort)
 * /tf: KHÔNG publish — do Pi xử lý (odom_to_tf_broadcaster.py)
 */

#include "MicroRosComm.h"
#include "RobotConfig.h"
#include "RobotMaster.h"
#include "TaskManager.h"
#include "IMUDriver.h"
#include <Arduino.h>
#include <math.h>

// ============================================================
// Helper: Euler sang Quaternion cho ROS 2
// ============================================================
static void eulerToQuaternion(float roll, float pitch, float yaw, float *q) {
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);

    q[0] = cy * cp * sr - sy * sp * cr; // X
    q[1] = sy * cp * sr + cy * sp * cr; // Y
    q[2] = sy * cp * cr - cy * sp * sr; // Z
    q[3] = cy * cp * cr + sy * sp * sr; // W
}

extern RobotMaster robotMaster;

// ============================================================
// CUSTOM TRANSPORT CHO USB CDC (Fix Bug 11: USBCDC Deadlock)
// ============================================================
#include <uxr/client/profile/transport/custom/custom_transport.h>

// Forward declare accessor cho telemetry (định nghĩa trong TaskMicroROS.cpp)
extern MicroRosComm* getMicroRosComm();

extern "C" {
    bool my_transport_open(struct uxrCustomTransport * transport) { return true; }
    bool my_transport_close(struct uxrCustomTransport * transport) { return true; }

    size_t my_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode) {
        Stream * stream = (Stream *) transport->args;
        size_t sent = 0;
        unsigned long start_time = millis();
        unsigned long last_progress = start_time;
        const unsigned long TOTAL_TIMEOUT_MS = 200; // Tổng thời gian tối đa để gửi frame

        while (sent < len) {
            unsigned long now = millis();
            unsigned long total_time = now - start_time;
            unsigned long no_progress = now - last_progress;

            // Kiểm tra deadline tổng và deadline progress ở mọi vòng lặp
            if (no_progress > TRANSPORT_WRITE_TIMEOUT_MS || total_time > TOTAL_TIMEOUT_MS) {
                MicroRosComm* uros = getMicroRosComm();
                if (uros) {
                    uros->telemetry.transport_write_timeouts++;
                    if (no_progress > uros->telemetry.max_no_progress_ms) {
                        uros->telemetry.max_no_progress_ms = no_progress;
                    }
                }
                if (errcode) *errcode = 1;
                break;
            }

            size_t space = stream->availableForWrite();
            if (space > 0) {
                size_t to_write = (len - sent < space) ? (len - sent) : space;
                size_t w = stream->write(buf + sent, to_write);
                if (w == 0) {
                    // Zero-byte write: driver trả 0 dù có space → hardware fault
                    MicroRosComm* uros = getMicroRosComm();
                    if (uros) {
                        uros->telemetry.zero_byte_writes++;
                        uros->telemetry.transport_fault = true;
                        uros->telemetry.last_fault_code = 3; // zero-byte
                        uros->telemetry.partial_write_requested = len;
                        uros->telemetry.partial_write_actual = sent;
                    }
                    if (errcode) *errcode = 1;
                    break;
                }
                sent += w;
                last_progress = millis(); // Reset no-progress timer
            } else {
                vTaskDelay(pdMS_TO_TICKS(1)); // YIELD CPU, không busy-wait
            }
        }

        // Kiểm tra partial write: đã gửi một phần nhưng không đủ frame
        // XRCE frame bị cắt giữa chừng → session bị hỏng vĩnh viễn
        if (sent > 0 && sent < len) {
            MicroRosComm* uros = getMicroRosComm();
            // CHÚ Ý: không ghi đè nếu fault_code=3 (zero byte)
            if (uros) {
                uros->telemetry.transport_partial_writes++;
                uros->telemetry.partial_write_requested = len;
                uros->telemetry.partial_write_actual = sent;
                uros->telemetry.transport_fault = true;
                if (uros->telemetry.last_fault_code != 3) {
                    uros->telemetry.last_fault_code = 2; // partial
                }
            }
            if (errcode) *errcode = 1;
        } else if (sent == 0 && len > 0) {
            // Không gửi được byte nào: cũng đặt fault
            MicroRosComm* uros = getMicroRosComm();
            if (uros && !uros->telemetry.transport_fault) {
                uros->telemetry.transport_fault = true;
                uros->telemetry.last_fault_code = 1; // timeout, 0 sent
            }
            if (errcode) *errcode = 1;
        }

        return sent;
    }

    size_t my_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode) {
        (void)errcode;
        Stream * stream = (Stream *) transport->args;
        size_t read_bytes = 0;
        unsigned long start_time = millis();

        while (read_bytes < len) {
            size_t avail = stream->available();
            if (avail > 0) {
                size_t to_read = (len - read_bytes < avail) ? (len - read_bytes) : avail;
                // Gọi readBytes với timeout=0 để lấy ngay lập tức dữ liệu đã có trong buffer
                stream->setTimeout(0);
                size_t r = stream->readBytes((char*)(buf + read_bytes), to_read);
                read_bytes += r;
            } else {
                if (millis() - start_time >= (unsigned long)timeout) {
                    // [QA6] Đếm read timeout
                    MicroRosComm* uros = getMicroRosComm();
                    if (uros) {
                        uros->telemetry.transport_read_timeouts++;
                    }
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(1)); // YIELD CPU CHO IDLE0 -> FIX WATCHDOG KHI MẤT KẾT NỐI
            }
        }
        return read_bytes;
    }
}


// ============================================================
// Macro kiem tra ket qua rcl — log loi neu fail
// ============================================================
#define RCCHECK(fn, msg) { \
    rcl_ret_t rc = (fn); \
    if (rc != RCL_RET_OK) { \
        DBG.printf("[uROS][ERROR] %s failed: %d\n", msg, (int)rc); \
        return false; \
    } \
}

// ============================================================
// INIT — Setup transport, thu ket noi Agent
// ============================================================
bool MicroRosComm::init() {
    // Setup Serial transport bằng custom transport an toàn (có yield CPU)
    rmw_uros_set_custom_transport(
        true,
        (void *) &Serial,
        my_transport_open,
        my_transport_close,
        my_transport_write,
        my_transport_read
    );

    DBG.println("[uROS] Transport configured: Serial (USB CDC) with WDT-Safe Custom Writer");

    // Reset error counters
    publish_fail_count_ = 0;
    executor_fail_count_ = 0;

    state_ = UROS_WAITING_AGENT;
    return true;
}

// ============================================================
// SPIN ONCE — Goi moi 20ms tu Task_MicroROS
// ============================================================
void MicroRosComm::spinOnce() {
    switch (state_) {

    // --- WAITING: Cho Agent san sang ---
    case UROS_WAITING_AGENT: {
        // [BUG14 FIX] Xóa sạch rác trong bộ đệm RX trước khi Ping!
        // Nếu không, các gói cmd_vel cũ sẽ làm ngập luồng đọc, khiến gói PONG bị rớt.
        while (Serial.available()) {
            Serial.read();
        }

        // Ping Agent voi timeout 500ms de de dang ket noi hon
        rcl_ret_t rc = rmw_uros_ping_agent(500, 1);

        if (rc == RCL_RET_OK) {
            DBG.println("[uROS] Agent detected! Creating entities...");
            if (createEntities_()) {
                state_ = UROS_CONNECTED;
                // Reset error counters khi kết nối thành công
                publish_fail_count_ = 0;
                executor_fail_count_ = 0;
                DBG.println("[uROS][OK] Node 'motion_slave' connected.");
            } else {
                DBG.println("[uROS][ERROR] Failed to create entities. Retrying...");
                destroyEntities_();
                // Van giu WAITING_AGENT de retry o lan spinOnce tiep theo
            }
        }
        // Neu Agent chua san sang → im lang, khong spam log
        break;
    }

    // --- CONNECTED: Spin executor xu ly callbacks ---
    case UROS_CONNECTED: {
        // [BUG17 FIX] Dọn sạch hàng đợi nhanh chóng bằng vòng lặp nhỏ (chống độ trễ/đơ xe)
        // Thay vì chỉ đọc 1 tin nhắn/chu kỳ (gây ùn tắc RX Buffer làm Agent bị kẹt/đơ),
        // Ta đọc tối đa 5 tin nhắn (timeout 1ms) = mất tối đa 5ms.
        // Năng lực xử lý: 5 messages * (1000/20ms) = 250 Hz, hoàn toàn bao tiêu được teleop 30Hz!
        bool any_executor_fail = false;
        for (int i = 0; i < 5; i++) {
            rcl_ret_t ret = rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));
            // RCL_RET_OK và RCL_RET_TIMEOUT đều không phải lỗi nghiêm trọng.
            // Chỉ đếm lỗi khi return code khác (ví dụ session corrupt, node invalid).
            if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
                any_executor_fail = true;
            }
        }

        // [Finding 2] Theo dõi executor fail liên tiếp → disconnect để reconnect
        if (any_executor_fail) {
            executor_fail_count_++;
            // [QA6] Đếm executor spin fail
            telemetry.executor_spin_fails++;
            if (executor_fail_count_ >= MAX_EXECUTOR_FAILS) {
                telemetry.executor_last_err = RCL_RET_ERROR;
                DBG.printf("[uROS][ERROR] Executor failed %d times, disconnecting...\n",
                           executor_fail_count_);
                state_ = UROS_DISCONNECTED;
            }
        } else {
            executor_fail_count_ = 0; // Reset khi thành công
        }

        // [BUG13 REVERTED] KHÔNG gọi rmw_uros_sync_session() khi đang CONNECTED!
        // Lý do: sync gửi request qua USB Serial, nếu response bị lỗi/trễ,
        // session XRCE-DDS bị corrupt → /odom ngừng publish → xe đứng im trong RViz.
        // Sync 1 lần lúc boot (trong createEntities_) là ĐỦ cho robot chạy ngắn.

        // [BUG18 REVERTED] KHÔNG dùng rmw_uros_ping_agent() khi đang CONNECTED!
        // Lý do: Khi ESP32 đang publish liên tục, đường truyền USB Serial bận rộn,
        // ping 10ms không đủ thời gian nhận PONG → false positive.
        // Cơ chế an toàn hiện tại đã ĐỦ MẠNH:
        // - RobotMaster Watchdog: 1000ms không nhận cmd_vel → EMERGENCY (phanh xe)
        // - Safety Watchdog trên Pi: 1000ms → gửi STOP
        // → Xe luôn dừng an toàn mà KHÔNG cần reboot ESP32.
        break;
    }

    // --- DISCONNECTED: Cleanup va quay lai WAITING ---
    case UROS_DISCONNECTED: {
        destroyEntities_();
        state_ = UROS_WAITING_AGENT;
        DBG.println("[uROS] Entities destroyed. Waiting for Agent...");
        break;
    }

    } // end switch
}

// ============================================================
// CREATE ENTITIES — Tao Allocator, Support, Node, Executor
// ============================================================
bool MicroRosComm::createEntities_() {
    init_state_ = 0;

    // 1. Allocator (su dung default allocator cua micro-ROS)
    allocator_ = rcl_get_default_allocator();

    // 2. Support (init rcl + rmw)
    RCCHECK(
        rclc_support_init(&support_, 0, nullptr, &allocator_),
        "rclc_support_init"
    );
    init_state_ = 1;

    // [BUG12 FIX] Đồng bộ thời gian thực (Time Sync) với Raspberry Pi!
    // BẮT BUỘC phải có, nếu không timestamp của /odom gửi từ ESP32 sẽ là năm 1970.
    if (rmw_uros_sync_session(1000) != RCL_RET_OK) {
        DBG.println("[uROS][WARN] Time sync failed, using local time.");
    } else {
        DBG.println("[uROS][OK] Time synced with Agent.");
    }

    // 3. Node: ten "motion_slave", namespace ""
    RCCHECK(
        rclc_node_init_default(&node_, "motion_slave", "", &support_),
        "rclc_node_init_default"
    );
    init_state_ = 2;

    // 3.1. Publisher: /odom (Best Effort)
    RCCHECK(
        rclc_publisher_init_best_effort(&pub_odom_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom"),
        "rclc_publisher_init_best_effort odom"
    );
    init_state_ = 3;

    // 3.2. Publisher: /imu/data (Best Effort)
    RCCHECK(
        rclc_publisher_init_best_effort(&pub_imu_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data"),
        "rclc_publisher_init_best_effort imu"
    );
    init_state_ = 4;

    // 3.3. Publisher: /env_status (Best Effort)
    RCCHECK(
        rclc_publisher_init_best_effort(&pub_env_status_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/env_status"),
        "rclc_publisher_init_best_effort env_status"
    );
    init_state_ = 5;

    // 3.5. Subscriber: /cmd_vel (BEST_EFFORT)
    RCCHECK(
        rclc_subscription_init_best_effort(&sub_cmd_vel_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"),
        "rclc_subscription_init_best_effort cmd_vel"
    );
    init_state_ = 6;

    // 3.6. Subscriber: /fire_target (Reliable)
    RCCHECK(
        rclc_subscription_init_default(&sub_fire_target_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "/fire_target"),
        "rclc_subscription_init_default fire_target"
    );
    init_state_ = 7;

    // 3.7. Subscriber: /pump_cmd (Reliable)
    RCCHECK(
        rclc_subscription_init_default(&sub_pump_cmd_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/pump_cmd"),
        "rclc_subscription_init_default pump_cmd"
    );
    init_state_ = 8;

    // Gan buffer tinh cho string message de tranh malloc
    msg_env_status_.data.data = env_status_buffer_;
    msg_env_status_.data.capacity = sizeof(env_status_buffer_);
    msg_env_status_.data.size = 0;

    // 4. Executor
    RCCHECK(
        rclc_executor_init(&executor_, &support_.context, EXECUTOR_HANDLES, &allocator_),
        "rclc_executor_init"
    );
    init_state_ = 9;

    // Them subscribers vao executor
    RCCHECK(
        rclc_executor_add_subscription(&executor_, &sub_cmd_vel_, &msg_cmd_vel_, &MicroRosComm::cmdVelCallback, ON_NEW_DATA),
        "rclc_executor_add_subscription cmd_vel"
    );
    RCCHECK(
        rclc_executor_add_subscription(&executor_, &sub_fire_target_, &msg_fire_target_, &MicroRosComm::fireTargetCallback, ON_NEW_DATA),
        "rclc_executor_add_subscription fire_target"
    );
    RCCHECK(
        rclc_executor_add_subscription(&executor_, &sub_pump_cmd_, &msg_pump_cmd_, &MicroRosComm::pumpCmdCallback, ON_NEW_DATA),
        "rclc_executor_add_subscription pump_cmd"
    );

    return true;
}

// ============================================================
// DESTROY ENTITIES — Cleanup truoc khi reconnect
// ============================================================
void MicroRosComm::destroyEntities_() {
    // Huy theo thu tu nguoc voi tao, chi huy nhung entity da khoi tao thanh cong
    rcl_ret_t rc;
    if (init_state_ >= 1) {
        // [QA6 FIX] Vô hiệu hóa timeout của destroy session, để không treo khi Agent mất
        rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support_.context);
        if (rmw_context) {
            rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
        }
    }

    if (init_state_ >= 9) { rc = rclc_executor_fini(&executor_); (void)rc; }
    if (init_state_ >= 8) { rc = rcl_subscription_fini(&sub_pump_cmd_, &node_); (void)rc; }
    if (init_state_ >= 7) { rc = rcl_subscription_fini(&sub_fire_target_, &node_); (void)rc; }
    if (init_state_ >= 6) { rc = rcl_subscription_fini(&sub_cmd_vel_, &node_); (void)rc; }
    if (init_state_ >= 5) { rc = rcl_publisher_fini(&pub_env_status_, &node_); (void)rc; }
    if (init_state_ >= 4) { rc = rcl_publisher_fini(&pub_imu_, &node_); (void)rc; }
    if (init_state_ >= 3) { rc = rcl_publisher_fini(&pub_odom_, &node_); (void)rc; }
    if (init_state_ >= 2) { rc = rcl_node_fini(&node_); (void)rc; }
    if (init_state_ >= 1) { rc = rclc_support_fini(&support_); (void)rc; }

    init_state_ = 0;
    publish_fail_count_ = 0;
    executor_fail_count_ = 0;

    // [QA6-B1] Clear transport fault flag để session mới bắt đầu sạch.
    // Không reset tổng counters (partial_writes, wr_timeouts, etc.) để giữ telemetry tích lũy.
    telemetry.transport_fault = false;

    // Flush USB CDC RX buffer: xóa XRCE fragments cũ còn sót
    while (Serial.available()) {
        Serial.read();
    }
}

// ============================================================
// PUBLISH DATA — Goi tu Task_MicroROS
// Return: true neu /odom va /imu publish OK, false neu fail
// ============================================================
bool MicroRosComm::publishData() {
    SharedContext* ctx = TaskManager_getContext();
    if (!ctx) return false;

    // Lấy dữ liệu Odom an toàn luồng
    OdometryData_t odom_data;
    xSemaphoreTake(ctx->stateMutex, portMAX_DELAY);
    odom_data = ctx->odom;
    xSemaphoreGive(ctx->stateMutex);

    // ----------------------------------------------------
    // Xây dựng msg_odom_
    // ----------------------------------------------------
    // Dùng thời gian thực từ Agent (đã sync) thay vì millis()
    int64_t epoch_ns = rmw_uros_epoch_nanos();
    msg_odom_.header.stamp.sec = (int32_t)(epoch_ns / 1000000000LL);
    msg_odom_.header.stamp.nanosec = (uint32_t)(epoch_ns % 1000000000LL);

    msg_odom_.header.frame_id.data = (char*)"odom";
    msg_odom_.header.frame_id.size = strlen(msg_odom_.header.frame_id.data);
    msg_odom_.header.frame_id.capacity = msg_odom_.header.frame_id.size + 1;

    msg_odom_.child_frame_id.data = (char*)"base_link";
    msg_odom_.child_frame_id.size = strlen(msg_odom_.child_frame_id.data);
    msg_odom_.child_frame_id.capacity = msg_odom_.child_frame_id.size + 1;

    msg_odom_.pose.pose.position.x = odom_data.x;
    msg_odom_.pose.pose.position.y = odom_data.y;
    msg_odom_.pose.pose.position.z = 0.0;

    float q[4];
    // Xe di chuyển trên mặt phẳng 2D, Roll và Pitch mặc định = 0
    eulerToQuaternion(0.0f, 0.0f, odom_data.yaw, q);
    msg_odom_.pose.pose.orientation.x = q[0];
    msg_odom_.pose.pose.orientation.y = q[1];
    msg_odom_.pose.pose.orientation.z = q[2];
    msg_odom_.pose.pose.orientation.w = q[3];

    msg_odom_.twist.twist.linear.x = odom_data.linear_velocity;
    msg_odom_.twist.twist.angular.z = odom_data.angular_velocity;

    // ----------------------------------------------------
    // Xây dựng msg_imu_
    // ----------------------------------------------------
    msg_imu_.header.stamp = msg_odom_.header.stamp;
    // [Finding 2 FIX] frame_id phải khớp URDF: imu_frame (không phải imu_link)
    msg_imu_.header.frame_id.data = (char*)"imu_frame";
    msg_imu_.header.frame_id.size = strlen(msg_imu_.header.frame_id.data);
    msg_imu_.header.frame_id.capacity = msg_imu_.header.frame_id.size + 1;

    msg_imu_.orientation.x = q[0];
    msg_imu_.orientation.y = q[1];
    msg_imu_.orientation.z = q[2];
    msg_imu_.orientation.w = q[3];

    msg_imu_.angular_velocity.x = 0.0; // Bỏ qua roll rate vì IMU chỉ chạy 2D hiện tại
    msg_imu_.angular_velocity.y = 0.0; // Bỏ qua pitch rate
    msg_imu_.angular_velocity.z = odom_data.angular_velocity;

    // [Finding 2] Publish /odom, /imu và kiểm tra return code

    // [QA6-B1 FIX] Không publish message tiếp theo khi session đã fault
    // 1. Publish odom
    rcl_ret_t rc_odom = RCL_RET_ERROR;
    if (!telemetry.transport_fault) {
        telemetry.odom_pub_attempts++;
        rc_odom = rcl_publish(&pub_odom_, &msg_odom_, nullptr);
        if (telemetry.transport_fault) {
            telemetry.odom_pub_fail++;
            telemetry.odom_last_err = rc_odom;
        } else if (rc_odom == RCL_RET_OK) {
            telemetry.odom_pub_ok++;
        } else {
            telemetry.odom_pub_fail++;
            telemetry.odom_last_err = rc_odom;
        }
    }

    // 2. Publish imu (chỉ khi chưa fault)
    rcl_ret_t rc_imu = RCL_RET_ERROR;
    if (!telemetry.transport_fault) {
        telemetry.imu_pub_attempts++;
        rc_imu = rcl_publish(&pub_imu_, &msg_imu_, nullptr);
        if (telemetry.transport_fault) {
            telemetry.imu_pub_fail++;
            telemetry.imu_last_err = rc_imu;
        } else if (rc_imu == RCL_RET_OK) {
            telemetry.imu_pub_ok++;
        } else {
            telemetry.imu_pub_fail++;
            telemetry.imu_last_err = rc_imu;
        }
    }

    // [QA6-B1 FIX] Kiểm tra transport fault SAU rcl_publish.
    // rcl_publish có thể trả OK vì nó chỉ ghi vào buffer nội bộ,
    // nhưng transport đã gửi thiếu frame → session XRCE hỏng.
    // KHÔNG được coi publish là thành công nếu transport_fault == true.
    if (telemetry.transport_fault) {
        telemetry.reconnects_by_transport++;
        DBG.printf("[uROS][FAULT] Transport fault detected! "
                   "fault_code=%d partial=%lu wr_timeout=%lu zero=%lu\n",
                   telemetry.last_fault_code,
                   telemetry.transport_partial_writes,
                   telemetry.transport_write_timeouts,
                   telemetry.zero_byte_writes);
        DBG.printf("[uROS][FAULT] Last partial: req=%u sent=%u | "
                   "reconnects_by_transport=%lu\n",
                   (unsigned)telemetry.partial_write_requested,
                   (unsigned)telemetry.partial_write_actual,
                   telemetry.reconnects_by_transport);
        // Dừng session hiện tại, chuyển sang DISCONNECTED để cleanup và tạo lại
        state_ = UROS_DISCONNECTED;
        return false;
    }

    // Theo dõi lỗi publish liên tiếp (rcl level)
    if (rc_odom != RCL_RET_OK || rc_imu != RCL_RET_OK) {
        publish_fail_count_++;
        if (publish_fail_count_ == 1) {
            DBG.printf("[uROS][WARN] Publish fail: odom=%d imu=%d (count=%d)\n",
                       (int)rc_odom, (int)rc_imu, publish_fail_count_);
        }
        if (publish_fail_count_ >= MAX_PUBLISH_FAILS) {
            DBG.printf("[uROS][ERROR] Publish failed %d times, disconnecting...\n",
                       publish_fail_count_);
            state_ = UROS_DISCONNECTED;
            return false;
        }
    } else {
        publish_fail_count_ = 0;
    }

    // ----------------------------------------------------
    // Xây dựng và Publish msg_env_status_ chỉ 2Hz (mỗi 10 vòng của 10Hz)
    // ----------------------------------------------------
    static uint8_t env_divider = 0;
    if (++env_divider >= 10) {
        env_divider = 0;

        // [Finding 6 FIX] Đọc sensorDataValid VÀ lastSensorData trong cùng critical section
        bool dataValid;
        SensorPacket_t env_data;
        xSemaphoreTake(ctx->stateMutex, portMAX_DELAY);
        dataValid = ctx->sensorDataValid;
        if (dataValid) {
            env_data = ctx->lastSensorData;
        }
        xSemaphoreGive(ctx->stateMutex);

        if (dataValid) {
            char fire_str[4];
            snprintf(fire_str, sizeof(fire_str), "%03d", env_data.fire_flags);

            int len = snprintf(env_status_buffer_, sizeof(env_status_buffer_),
                "{\"fire\":\"%s\",\"gas\":%.1f,\"temp\":%.1f,\"batt\":%.1f}",
                fire_str, env_data.gas_ppm, env_data.temperature, env_data.batt_voltage);

            if (len > 0 && len < (int)sizeof(env_status_buffer_)) {
                msg_env_status_.data.size = len;
                if (!telemetry.transport_fault) {
                    rcl_ret_t rc_env = rcl_publish(&pub_env_status_, &msg_env_status_, nullptr);
                    (void)rc_env;
                    if (telemetry.transport_fault) {
                        telemetry.reconnects_by_transport++;
                        state_ = UROS_DISCONNECTED;
                        return false;
                    }
                }
                // env_status fail không gây disconnect (dữ liệu không critical)
            }
        } else {
            int len = snprintf(env_status_buffer_, sizeof(env_status_buffer_), "{\"status\":\"WROOM_OFFLINE\"}");
            msg_env_status_.data.size = len;
            if (!telemetry.transport_fault) {
                rcl_ret_t rc_env = rcl_publish(&pub_env_status_, &msg_env_status_, nullptr);
                (void)rc_env;
                if (telemetry.transport_fault) {
                    telemetry.reconnects_by_transport++;
                    state_ = UROS_DISCONNECTED;
                    return false;
                }
            }
        }
    }

    return true;
}

// ============================================================
// CALLBACK: Nhan lenh dieu khien tu /cmd_vel
// ============================================================
void MicroRosComm::cmdVelCallback(const void* msgin) {
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
    if (!msg) return;

    SharedContext* ctx = TaskManager_getContext();
    if (!ctx) return;

    // Cập nhật lệnh vận tốc và bảo vệ bằng Mutex
    xSemaphoreTake(ctx->cmdMutex, portMAX_DELAY);
    ctx->cmdVel.target_vx = msg->linear.x;
    ctx->cmdVel.target_wz = msg->angular.z;
    xSemaphoreGive(ctx->cmdMutex);

    // Reset watchdog và báo cho RobotMaster biết đã nhận lệnh
    robotMaster.notifyCmdReceived();

    // [QA6] Ghi timestamp nhận cmd_vel
    MicroRosComm* uros = getMicroRosComm();
    if (uros) {
        uros->telemetry.last_cmd_vel_ms = millis();
    }
}

// ============================================================
// Biến static lưu trạng thái cơ cấu chấp hành để không bị mất
// ============================================================
static uint8_t s_servo_pan = 90;
static uint8_t s_pump_on = 0;
static uint8_t s_buzzer_on = 0;

// ============================================================
// CALLBACK: Nhan toa do muc tieu lua /fire_target
// ============================================================
void MicroRosComm::fireTargetCallback(const void* msgin) {
    const geometry_msgs__msg__Point* msg = (const geometry_msgs__msg__Point*)msgin;
    if (!msg) return;

    SharedContext* ctx = TaskManager_getContext();
    if (!ctx) return;

    // x nam trong khoang [-1.0, 1.0]. Can chuyen sang [0, 180] do
    // Vi du: x = 0.0 -> 90 do, x = 0.5 -> 135 do, x = -0.5 -> 45 do
    float pan_f = 90.0f + (msg->x * 90.0f);
    if (pan_f > 180.0f) pan_f = 180.0f;
    if (pan_f < 0.0f) pan_f = 0.0f;

    s_servo_pan = (uint8_t)pan_f;
    DBG.printf("[ACT] Servo Pan -> %d do\n", s_servo_pan);

    // [Finding 1 FIX] xQueueOverwrite cho queue length 1 — lệnh mới luôn ghi đè lệnh cũ
    if (ctx->actuatorQueue != nullptr) {
        ActuatorCmd_t cmd = {ACTUATOR_CMD_HEADER, s_servo_pan, s_pump_on, s_buzzer_on, 0};
        xQueueOverwrite(ctx->actuatorQueue, &cmd);
    }
}

// ============================================================
// CALLBACK: Nhan lenh bom nuoc /pump_cmd
// ============================================================
void MicroRosComm::pumpCmdCallback(const void* msgin) {
    const std_msgs__msg__Bool* msg = (const std_msgs__msg__Bool*)msgin;
    if (!msg) return;

    SharedContext* ctx = TaskManager_getContext();
    if (!ctx) return;

    s_pump_on = msg->data ? 1 : 0;
    DBG.printf("[ACT] Pump %s\n", s_pump_on ? "ON" : "OFF");

    // [Finding 1 FIX] xQueueOverwrite — đảm bảo pump OFF không bao giờ bị drop
    if (ctx->actuatorQueue != nullptr) {
        ActuatorCmd_t cmd = {ACTUATOR_CMD_HEADER, s_servo_pan, s_pump_on, s_buzzer_on, 0};
        xQueueOverwrite(ctx->actuatorQueue, &cmd);
    }
}
