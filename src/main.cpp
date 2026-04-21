#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Forward declarations of task functions
void Task_Motion(void *pvParameters);
void Task_MicroROS(void *pvParameters);
void Task_Actuator(void *pvParameters);
void Task_System(void *pvParameters);

void setup() {
  Serial.begin(115200);
  
  // Khởi tạo các Hardware ngầm định hoặc các Module chức năng
  // VD: TaskManager::init(); hoặc các driver khác...
  
  // Tạo các Task trong FreeRTOS
  
  // TASK 1: Motion & Kinematics Control (Ưu tiên cao - Thực thi thời gian thực)
  xTaskCreatePinnedToCore(
    Task_Motion,          // Task function
    "MotionTask",         // Task name
    8192,                 // Stack size
    NULL,                 // Parameters
    5,                    // Priority (cao)
    NULL,                 // Task handle
    1                     // Core 1 (Core cho Ứng dụng)
  );

  // TASK 2: Giao tiếp micro-ROS (Ưu tiên trung bình)
  xTaskCreatePinnedToCore(
    Task_MicroROS,        // Task function
    "MicroRosTask",       // Task name
    16384,                // Stack size
    NULL,                 // Parameters
    3,                    // Priority
    NULL,                 // Task handle
    0                     // Core 0 (Thường dùng cho mạng / Wi-Fi)
  );

  // TASK 3: Điều khiển Actuators (Relay, Servo Pan-Tilt) (Ưu tiên trung bình)
  xTaskCreatePinnedToCore(
    Task_Actuator,        // Task function
    "ActuatorTask",       // Task name
    4096,                 // Stack size
    NULL,                 // Parameters
    3,                    // Priority
    NULL,                 // Task handle
    1                     // Core 1
  );

  // TASK 4: Máy trạng thái Hệ thống & Cảnh báo an toàn (Ưu tiên cao nhất cho System Failsafe)
  xTaskCreatePinnedToCore(
    Task_System,          // Task function
    "SystemTask",         // Task name
    4096,                 // Stack size
    NULL,                 // Parameters
    6,                    // Priority (cao nhất)
    NULL,                 // Task handle
    1                     // Core 1
  );
}

void loop() {
  // Không làm gì trong loop, nhường lại cho FreeRTOS schedulers quản lý các Task
  vTaskDelete(NULL);
}

// ---------------- CÁC FRAMEWORK HÀM TASK ------------------

// TASK 1: Điều khiển Di chuyển
void Task_Motion(void *pvParameters) {
  while (1) {
    // 1. Đọc dữ liệu Encoder
    // 2. Cập nhật Odometry & tính góc qua IMU
    // 3. Chạy thuật toán PID nhắm theo Setpoint vận tốc
    // 4. Xuất xung PWM ra mạch cầu H (MotorDriver)
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Loop rate: 100Hz (10ms)
  }
}

// TASK 2: Giao tiếp micro-ROS
void Task_MicroROS(void *pvParameters) {
  while (1) {
    // 1. Sync time với Agent
    // 2. Publish ODOM (Encoder + IMU) lên Raspberry Pi
    // 3. Spin các subscriber (nhận cmd_vel từ Nav2)
    
    vTaskDelay(pdMS_TO_TICKS(50)); // Loop rate: 20Hz (50ms)
  }
}

// TASK 3: Điều khiển Thực thi & Cảm biến phụ
void Task_Actuator(void *pvParameters) {
  while (1) {
    // 1. Xoay góc Servo tới tâm ngọn lửa
    // 2. Kích hoạt/Ngắt Relay bơm nước
    // 3. Kiểm tra thông số phụ nếu cần
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Loop rate: 10Hz (100ms)
  }
}

// TASK 4: Máy Trang Thái Chính & Giám sát Failsafe
void Task_System(void *pvParameters) {
  while (1) {
    // 1. Kiểm tra Health check mạng 4G/Tailscale
    // 2. Lắng nghe ghi đè từ Override NRF24L01+
    // 3. Quản lý Enum trạng thái chính của xe (Auto / Override / Emergency)
    
    vTaskDelay(pdMS_TO_TICKS(200)); // Loop rate: 5Hz (200ms)
  }
}