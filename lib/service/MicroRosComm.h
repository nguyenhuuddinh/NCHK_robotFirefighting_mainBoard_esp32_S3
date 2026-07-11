/**
 * @file MicroRosComm.h
 * @brief Module giao tiep micro-ROS cho ESP32-S3 Motion Slave
 *
 * Layer: service/ (logic giao tiep, khong truy cap GPIO)
 *
 * Chuc nang Phase 4.2:
 *   - Khoi tao Serial transport (USB Serial → Raspberry Pi Agent)
 *   - Tao Node "motion_slave" voi allocator
 *   - Cung cap API init/spinOnce de Task_MicroROS goi
 *
 * Ghi chu:
 *   - Transport su dung Serial (cung cong USB voi Serial Monitor)
 *   - micro-ROS Agent chay tren Raspberry Pi
 *   - Cac Publisher/Subscriber se duoc them o Phase 4.3+
 */

#pragma once

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_platformio.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>

/**
 * @brief Trang thai ket noi micro-ROS
 */
typedef enum {
    UROS_WAITING_AGENT = 0,  // Dang cho Agent san sang
    UROS_CONNECTED,          // Da ket noi, Node dang chay
    UROS_DISCONNECTED        // Mat ket noi (can reconnect)
} MicroRosState_t;

/**
 * @brief Module MicroRosComm
 *
 * Quan ly vong doi cua micro-ROS:
 *   WAITING_AGENT → CONNECTED ↔ DISCONNECTED
 *
 * Khong su dung dynamic allocation trong runtime.
 * Tat ca rcl objects duoc cap phat tinh (static member).
 */
class MicroRosComm {
public:
    /**
     * @brief Khoi tao transport va tao Node
     * @return true neu init thanh cong (Agent san sang)
     * @return false neu Agent chua san sang (se retry trong spinOnce)
     */
    bool init();

    /**
     * @brief Spin executor 1 lan (goi trong Task loop moi 20ms)
     *
     * Xu ly theo trang thai hien tai:
     *   - WAITING_AGENT: ping Agent, neu OK -> tao Node
     *   - CONNECTED: spin executor (xu ly callbacks)
     *   - DISCONNECTED: destroy + retry init
     */
    void spinOnce();

    /**
     * @brief Publish du lieu Odometry va IMU len ROS 2
     */
    void publishData();

    /**
     * @brief Lay trang thai hien tai cua micro-ROS
     */
    MicroRosState_t getState() const { return state_; }

    /**
     * @brief Kiem tra ket noi con song khong
     */
    bool isConnected() const { return state_ == UROS_CONNECTED; }

    // --- Accessors cho Task_MicroROS (Phase 4.3+ se them Publisher/Sub) ---
    rcl_node_t* getNode() { return &node_; }
    rclc_support_t* getSupport() { return &support_; }
    rcl_allocator_t* getAllocator() { return &allocator_; }
    rclc_executor_t* getExecutor() { return &executor_; }

private:
    // --- Trang thai ---
    MicroRosState_t state_ = UROS_WAITING_AGENT;

    // --- micro-ROS objects (cap phat tinh) ---
    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rcl_node_t node_;
    rclc_executor_t executor_;

    // --- Publishers ---
    rcl_publisher_t pub_odom_;
    rcl_publisher_t pub_imu_;
    nav_msgs__msg__Odometry msg_odom_;
    sensor_msgs__msg__Imu msg_imu_;

    // --- Retry/timeout ---
    static const int AGENT_PING_TIMEOUT_MS = 200;
    static const int EXECUTOR_HANDLES = 4; // Du cho: /cmd_vel, /fire_target, /pump_cmd + du phong

    /**
     * @brief Tao Node va Executor
     * @return true neu thanh cong
     */
    bool createEntities_();

    /**
     * @brief Huy toan bo rcl objects (truoc khi retry)
     */
    void destroyEntities_();
};
