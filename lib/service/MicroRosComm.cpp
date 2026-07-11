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
    // Setup Serial transport (su dung Serial = USB CDC → Raspberry Pi)
    // Baudrate da duoc set boi Serial.begin(115200) trong main.cpp
    set_microros_serial_transports(Serial);
    DBG.println("[uROS] Transport configured: Serial (USB CDC)");

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
        // Ping Agent voi timeout ngan (khong block task qua lau)
        rcl_ret_t rc = rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1);

        if (rc == RCL_RET_OK) {
            DBG.println("[uROS] Agent detected! Creating entities...");
            if (createEntities_()) {
                state_ = UROS_CONNECTED;
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
        // Spin executor voi timeout ngan (khong block task)
        rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(5));

        // Kiem tra Agent con song khong (moi ~50 lan spin = ~1 giay)
        static uint8_t ping_counter = 0;
        if (++ping_counter >= 50) {
            ping_counter = 0;
            rcl_ret_t rc = rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1);
            if (rc != RCL_RET_OK) {
                DBG.println("[uROS][WARN] Agent lost! Destroying entities & Triggering EMERGENCY...");
                state_ = UROS_DISCONNECTED;
                robotMaster.setState(STATE_EMERGENCY);
            }
        }
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
    // 1. Allocator (su dung default allocator cua micro-ROS)
    allocator_ = rcl_get_default_allocator();

    // 2. Support (init rcl + rmw)
    RCCHECK(
        rclc_support_init(&support_, 0, nullptr, &allocator_),
        "rclc_support_init"
    );

    // 3. Node: ten "motion_slave", namespace ""
    RCCHECK(
        rclc_node_init_default(&node_, "motion_slave", "", &support_),
        "rclc_node_init_default"
    );

    // 3.1. Publisher: /odom
    RCCHECK(
        rclc_publisher_init_default(&pub_odom_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom"),
        "rclc_publisher_init_default odom"
    );

    // 3.2. Publisher: /imu/data
    RCCHECK(
        rclc_publisher_init_default(&pub_imu_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data"),
        "rclc_publisher_init_default imu"
    );

    // 4. Executor (so handle = EXECUTOR_HANDLES, du cho subscriber tuong lai)
    RCCHECK(
        rclc_executor_init(&executor_, &support_.context, EXECUTOR_HANDLES, &allocator_),
        "rclc_executor_init"
    );

    return true;
}

// ============================================================
// DESTROY ENTITIES — Cleanup truoc khi reconnect
// ============================================================
void MicroRosComm::destroyEntities_() {
    // Huy theo thu tu nguoc voi tao
    rcl_ret_t rc;
    rc = rclc_executor_fini(&executor_); (void)rc;
    rc = rcl_publisher_fini(&pub_imu_, &node_); (void)rc;
    rc = rcl_publisher_fini(&pub_odom_, &node_); (void)rc;
    rc = rcl_node_fini(&node_);          (void)rc;
    rc = rclc_support_fini(&support_);   (void)rc;
}

// ============================================================
// PUBLISH DATA — Goi tu Task_MicroROS
// ============================================================
void MicroRosComm::publishData() {
    SharedContext* ctx = TaskManager_getContext();
    if (!ctx) return;

    // Lấy dữ liệu Odom an toàn luồng
    OdometryData_t odom_data;
    xSemaphoreTake(ctx->stateMutex, portMAX_DELAY);
    odom_data = ctx->odom;
    xSemaphoreGive(ctx->stateMutex);

    // ----------------------------------------------------
    // Xây dựng msg_odom_
    // ----------------------------------------------------
    int64_t current_time_ms = millis();
    msg_odom_.header.stamp.sec = current_time_ms / 1000;
    msg_odom_.header.stamp.nanosec = (current_time_ms % 1000) * 1000000;
    
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
    msg_imu_.header.frame_id.data = (char*)"imu_link";
    msg_imu_.header.frame_id.size = strlen(msg_imu_.header.frame_id.data);
    msg_imu_.header.frame_id.capacity = msg_imu_.header.frame_id.size + 1;

    msg_imu_.orientation.x = q[0];
    msg_imu_.orientation.y = q[1];
    msg_imu_.orientation.z = q[2];
    msg_imu_.orientation.w = q[3];

    msg_imu_.angular_velocity.x = 0.0; // Bỏ qua roll rate vì IMU chỉ chạy 2D hiện tại
    msg_imu_.angular_velocity.y = 0.0; // Bỏ qua pitch rate
    msg_imu_.angular_velocity.z = odom_data.angular_velocity;

    // Publish
    rcl_publish(&pub_odom_, &msg_odom_, nullptr);
    rcl_publish(&pub_imu_, &msg_imu_, nullptr);
}
