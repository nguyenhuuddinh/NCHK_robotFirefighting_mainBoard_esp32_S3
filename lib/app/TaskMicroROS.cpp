/**
 * @file TaskMicroROS.cpp
 * @brief FreeRTOS Task cho micro-ROS (Core 0)
 *
 * Chay tren Core 0, Priority 3 (sau Task_Motion, truoc UART va Logger)
 * Goi MicroRosComm::spinOnce() moi 20ms (50Hz)
 *
 * Publish: /odom, /imu/data, /env_status o 10Hz
 * /tf: KHONG publish tu ESP32 — do Pi xu ly (odom_to_tf_broadcaster.py)
 *
 * Stack: 8192 bytes (micro-ROS can nhieu hon binh thuong)
 */

#include "TaskManager.h"
#include "MicroRosComm.h"
#include "RobotConfig.h"
#include "RobotMaster.h"
#include <Arduino.h>

extern RobotMaster robotMaster;

// Instance MicroRosComm (cap phat tinh, ton tai suot doi song chuong trinh)
static MicroRosComm g_microRos;

// Accessor cho cac module khac can truy cap
MicroRosComm* getMicroRosComm() {
    return &g_microRos;
}

void Task_MicroROS(void* pvParam) {
    // SharedContext* ctx = static_cast<SharedContext*>(pvParam);

    // Cho he thong on dinh truoc khi init micro-ROS
    vTaskDelay(pdMS_TO_TICKS(2000));

    DBG.println("[uROS] Task_MicroROS started on Core 0");

    // Khoi tao transport
    if (!g_microRos.init()) {
        DBG.println("[uROS][ERROR] Transport init failed! Task suspended.");
        vTaskSuspend(nullptr);
    }

    DBG.println("[uROS] Waiting for micro-ROS Agent...");

    // === Main Loop ===
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        // Cập nhật State Machine của Robot (để check Watchdog)
        robotMaster.update();

        // [QA CRITICAL FIX] Khi dang cho Agent (WAITING/DISCONNECTED), rmw_uros_ping_agent tốn 100-200ms.
        // Neu dung vTaskDelayUntil(20ms), Task se khong bao gio duoc ngu (do T_exec > T_period),
        // dan den chiem dung 100% Core 0 (Priority 3) -> IDLE0 bi chet doi -> Task Watchdog Reset xe!
        // -> Khi chua ket noi: Ping 2Hz (ngu 500ms) la du va an toan cho WDT.
        // -> Khi da ket noi: Chay 50Hz (20ms) bang vTaskDelay de publish mượt.
        if (g_microRos.getState() != UROS_CONNECTED) {
            g_microRos.spinOnce(); // Spin để tìm Agent hoặc cleanup DISCONNECTED
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            // [BUG10 FIX] ĐỌC /cmd_vel TRƯỚC, PUBLISH SAU!
            // Spin executor ĐẦU TIÊN để đọc /cmd_vel, rồi mới publish.
            // Dù publish block, /cmd_vel đã được xử lý an toàn rồi.
            g_microRos.spinOnce();

            // Nếu spinOnce đã chuyển sang DISCONNECTED (do executor fail),
            // bỏ qua publish và vào vòng tiếp theo để cleanup.
            if (g_microRos.getState() != UROS_CONNECTED) {
                vTaskDelay(pdMS_TO_TICKS(TASK_MICROROS_PERIOD_MS));
                continue;
            }

            // [QA6] Telemetry log mỗi ~5 giây (250 * 20ms) qua UART0/DBG
            // KHÔNG log lên Serial USB CDC (đang dùng làm XRCE transport)
            static int telemetry_divider = 0;
            if (++telemetry_divider >= 250) {
                telemetry_divider = 0;
                const MicroRosTelemetry& t = g_microRos.telemetry;
                uint32_t now_ms = millis();
                uint32_t cmd_age_ms = (t.last_cmd_vel_ms > 0) ? (now_ms - t.last_cmd_vel_ms) : 0;

                DBG.println("--- [uROS TELEMETRY] ---");
                DBG.printf("  odom: try=%lu ok=%lu fail=%lu lastErr=%d\n",
                    t.odom_pub_attempts, t.odom_pub_ok, t.odom_pub_fail, (int)t.odom_last_err);
                DBG.printf("  imu:  try=%lu ok=%lu fail=%lu lastErr=%d\n",
                    t.imu_pub_attempts, t.imu_pub_ok, t.imu_pub_fail, (int)t.imu_last_err);
                DBG.printf("  transport: rd_to=%lu wr_to=%lu partial=%lu zero=%lu\n",
                    t.transport_read_timeouts, t.transport_write_timeouts,
                    t.transport_partial_writes, t.zero_byte_writes);
                if (t.transport_partial_writes > 0) {
                    DBG.printf("  last_partial: req=%u sent=%u\n",
                        (unsigned)t.partial_write_requested, (unsigned)t.partial_write_actual);
                }
                DBG.printf("  fault: active=%d code=%d reconn=%lu max_noprog=%lums\n",
                    t.transport_fault ? 1 : 0, t.last_fault_code,
                    t.reconnects_by_transport, t.max_no_progress_ms);
                DBG.printf("  executor: spin_fail=%lu lastErr=%d\n",
                    t.executor_spin_fails, (int)t.executor_last_err);
                DBG.printf("  cmd_vel: last=%lums ago=%lums\n",
                    t.last_cmd_vel_ms, cmd_age_ms);
                DBG.printf("  heap: free=%u min=%u\n",
                    ESP.getFreeHeap(), ESP.getMinFreeHeap());
                DBG.printf("  stack_hwm: Task_uROS=%u words\n",
                    uxTaskGetStackHighWaterMark(nullptr));
                DBG.println("------------------------");
            }

            // Publish /odom, /imu ở 10Hz (mỗi 100ms)
            // SLAM Toolbox chỉ cần 5-10Hz là đủ (Lidar quét ~5-8Hz)
            static int publish_divider = 0;
            publish_divider++;
            if (publish_divider >= 5) { // 20ms * 5 = 100ms (10Hz)
                bool pub_ok = g_microRos.publishData();
                publish_divider = 0;

                // Nếu publishData() fail liên tiếp, nó sẽ tự set DISCONNECTED.
                // Vòng tiếp theo sẽ vào nhánh cleanup ở trên.
                if (!pub_ok && g_microRos.getState() == UROS_DISCONNECTED) {
                    DBG.println("[uROS] Publish triggered disconnect, will cleanup next cycle.");
                }
            }

            // Luôn ngủ 20ms để nhường CPU cho Core 0 (tránh Task WDT)
            vTaskDelay(pdMS_TO_TICKS(TASK_MICROROS_PERIOD_MS));
        }
    }
}
