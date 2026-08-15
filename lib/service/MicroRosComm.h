/**
 * @file MicroRosComm.h
 * @brief Module giao tiep micro-ROS cho ESP32-S3 Motion Slave
 *
 * Layer: service/ (logic giao tiep, khong truy cap GPIO)
 *
 * Chuc nang:
 *   - Khoi tao Serial transport (USB Serial → Raspberry Pi Agent)
 *   - Tao Node "motion_slave" voi allocator
 *   - Publish /odom, /imu/data, /env_status (Best Effort)
 *   - Subscribe /cmd_vel, /fire_target, /pump_cmd
 *   - Cung cap API init/spinOnce de Task_MicroROS goi
 *
 * Ghi chu:
 *   - /tf KHÔNG publish từ ESP32 — do Pi xử lý (odom_to_tf_broadcaster.py)
 *   - Transport su dung Serial (USB CDC)
 *   - micro-ROS Agent chay tren Raspberry Pi
 */

#pragma once

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_platformio.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/point.h>

// Hằng số transport — không hardcode magic number
static const unsigned long TRANSPORT_WRITE_TIMEOUT_MS = 50;

/**
 * @brief Telemetry counters cho chẩn đoán micro-ROS
 * Được log ra UART0/DBG mỗi ~5 giây, không log lên USB CDC.
 */
struct MicroRosTelemetry {
    // Publish counters
    uint32_t odom_pub_attempts = 0;
    uint32_t odom_pub_ok = 0;
    uint32_t odom_pub_fail = 0;
    rcl_ret_t odom_last_err = RCL_RET_OK;

    uint32_t imu_pub_attempts = 0;
    uint32_t imu_pub_ok = 0;
    uint32_t imu_pub_fail = 0;
    rcl_ret_t imu_last_err = RCL_RET_OK;

    // Transport counters
    uint32_t transport_read_timeouts = 0;
    uint32_t transport_write_timeouts = 0;
    uint32_t transport_partial_writes = 0;
    size_t   partial_write_requested = 0;   // kích thước yêu cầu lần partial gần nhất
    size_t   partial_write_actual = 0;      // số byte thực tế đã gửi

    // Transport fault tracking
    volatile bool transport_fault = false;   // true = session hỏng, cần reconnect
    uint32_t zero_byte_writes = 0;           // stream->write trả 0
    uint32_t reconnects_by_transport = 0;    // số lần reconnect do transport fault
    unsigned long max_no_progress_ms = 0;    // thời gian no-progress lớn nhất đã ghi
    int last_fault_code = 0;                 // mã lỗi gần nhất (1=timeout, 2=partial, 3=zero)

    // Executor counters
    uint32_t executor_spin_fails = 0;
    rcl_ret_t executor_last_err = RCL_RET_OK;

    // Callback timing
    uint32_t last_cmd_vel_ms = 0;  // millis() lúc nhận callback /cmd_vel gần nhất
};

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
 *
 * Publish: /odom (BE), /imu/data (BE), /env_status (BE)
 * Subscribe: /cmd_vel (BE), /fire_target (Reliable), /pump_cmd (Reliable)
 * /tf: KHÔNG publish — do Pi xử lý qua odom_to_tf_broadcaster.py
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
     * @brief Publish du lieu Odometry, IMU, env_status len ROS 2
     * @return true neu publish /odom va /imu thanh cong
     * @return false neu publish fail (caller nen kiem tra reconnect)
     */
    bool publishData();

    /**
     * @brief Lay trang thai hien tai cua micro-ROS
     */
    MicroRosState_t getState() const { return state_; }

    /**
     * @brief Kiem tra ket noi con song khong
     */
    bool isConnected() const { return state_ == UROS_CONNECTED; }

    // --- Accessors cho Task_MicroROS ---
    rcl_node_t* getNode() { return &node_; }
    rclc_support_t* getSupport() { return &support_; }
    rcl_allocator_t* getAllocator() { return &allocator_; }
    rclc_executor_t* getExecutor() { return &executor_; }

    // --- Telemetry (QA6: chẩn đoán /odom mất) ---
    MicroRosTelemetry telemetry;

private:
    // --- Trang thai ---
    MicroRosState_t state_ = UROS_WAITING_AGENT;
    uint8_t init_state_ = 0; // Theo doi tien do init (0-9) de destroy an toan

    // --- micro-ROS objects (cap phat tinh) ---
    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rcl_node_t node_;
    rclc_executor_t executor_;

    // --- Publishers (chi /odom, /imu, /env_status — khong co /tf) ---
    rcl_publisher_t pub_odom_;
    rcl_publisher_t pub_imu_;
    rcl_publisher_t pub_env_status_;

    nav_msgs__msg__Odometry msg_odom_;
    sensor_msgs__msg__Imu msg_imu_;
    std_msgs__msg__String msg_env_status_;

    // --- Subscribers ---
    rcl_subscription_t sub_cmd_vel_;
    geometry_msgs__msg__Twist msg_cmd_vel_;

    rcl_subscription_t sub_fire_target_;
    geometry_msgs__msg__Point msg_fire_target_;

    rcl_subscription_t sub_pump_cmd_;
    std_msgs__msg__Bool msg_pump_cmd_;

    // Buffer tĩnh cho chuỗi JSON env_status (tránh malloc)
    char env_status_buffer_[128];

    // --- Error tracking (Finding 2: detect publish/executor fail → reconnect) ---
    uint8_t publish_fail_count_ = 0;
    uint8_t executor_fail_count_ = 0;
    static const uint8_t MAX_PUBLISH_FAILS = 5;   // 5 lần fail liên tiếp → disconnect
    static const uint8_t MAX_EXECUTOR_FAILS = 10;  // 10 lần fail liên tiếp → disconnect

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

    // Callbacks
    static void cmdVelCallback(const void* msgin);
    static void fireTargetCallback(const void* msgin);
    static void pumpCmdCallback(const void* msgin);
};
