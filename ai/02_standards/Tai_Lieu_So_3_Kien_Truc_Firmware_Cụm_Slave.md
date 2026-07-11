# BẢN MÔ TẢ KIẾN TRÚC FIRMWARE: CỤM SLAVE ĐIỀU KHIỂN ĐỘNG LỰC VÀ CẢM BIẾN (ESP32-S3 & ESP32-WROOM)

## 1. TỔNG QUAN KIẾN TRÚC PHÂN TÁN (DISTRIBUTED FIRMWARE ARCHITECTURE)
Hệ thống cấp thấp (Low-level Control) được phân tách thành 2 vi điều khiển chạy độc lập trên nền tảng FreeRTOS.
* **ESP32-S3 (Motion Slave):** Đóng vai trò là một micro-ROS Node chính thức, giao tiếp trực tiếp với Raspberry Pi. Tập trung toàn bộ tài nguyên tính toán cho hệ chuyển động.
* **ESP32-WROOM (Sensor & Actuator Slave):** Đóng vai trò là Trạm thu thập cảm biến và điều khiển cơ cấu chấp hành (Servo Pan vòi phun, Relay bơm, Buzzer), giao tiếp với ESP32-S3 thông qua UART.

## 2. KIẾN TRÚC FIRMWARE: ESP32-S3 (MOTION SLAVE)
Sử dụng sức mạnh Dual-core của lõi Xtensa LX7 để duy trì tần số cập nhật Odometry ở mức > 50Hz.

### 2.1. Phân bổ Task (FreeRTOS Core Allocation)
* 🔴 **Core 1 (Motion Core - Ưu tiên Cao):**
    * **Task_Motion (20ms):** Đọc tích lũy 4 bộ Encoder phần cứng (PCNT). Đọc MPU6050 qua I2C (giá trị đã cache). Tính toán Động học Lái trượt (Skid-Steer Kinematics) để chuyển đổi từ $(V_x, \omega)$ của ROS 2 sang tốc độ 4 bánh, chạy Feed-Forward + PID và xuất Hardware PWM. Cập nhật Odometry $(x, y, \theta)$.
* 🔵 **Core 0 (Comms Core - Ưu tiên Trung bình):**
    * **Task_MicroROS (20ms):** Chạy Executor của micro-ROS, Publish Topic `/odom`, `/imu/data`, `/env_status` lên Pi và Subscribe Topic `/cmd_vel`, `/fire_target`, `/pump_cmd` từ Pi.
    * **Task_UART_Rx (50ms):** Nhận gói tin trạng thái từ WROOM (Dữ liệu lửa, gas, nhiệt độ, pin) và đẩy vào hàng đợi (Queue) để micro-ROS Publish tiếp lên Pi. Gửi lệnh điều khiển Actuator (servo pan, bơm, buzzer) xuống WROOM.

### 2.2. Xử lý Động học Lái trượt (Skid-Steer)
Xe 4 bánh cao su khi xoay tại chỗ sẽ sinh ra độ trượt (slip) rất lớn. Firmware kết hợp **Vận tốc tính từ Encoder** và **Vận tốc góc Yaw từ IMU** (Sensor Fusion) bằng Complementary Filter thích ứng để tính ra tọa độ Odometry $(x, y, \theta)$ thực tế của xe:
* Khi xoay ($|\omega| > 0.1$ rad/s): $\alpha = 0.995$ (tin gyro gần 100% vì encoder trượt nhiều).
* Khi đi thẳng: $\alpha = 0.95$ (blend đều gyro + encoder).
* Công thức: $\omega_{fused} = \alpha \cdot \omega_{gyro} + (1 - \alpha) \cdot \omega_{encoder}$

## 3. KIẾN TRÚC FIRMWARE: ESP32-WROOM (SENSOR & ACTUATOR SLAVE)
Firmware này hoạt động theo cơ chế thu thập dữ liệu định kỳ và truyền về S3 qua UART.

### 3.1. Phân bổ Task (FreeRTOS Core Allocation)
* 🔴 **Core 1 (Actuator Core - Ưu tiên Cao):**
    * **Task_Actuator (50ms):** Điều khiển Servo Pan xoay vòi phun nước theo phương ngang, Relay bơm nước và Buzzer cảnh báo dựa trên lệnh nhận từ ESP32-S3 qua UART.
    * **Task_UART_Rx (50ms):** Nhận gói tin lệnh `ActuatorCmd_t` từ S3, parse + thực thi.
* 🔵 **Core 0 (Sensor Core - Ưu tiên Trung bình):**
    * **Task_Sensor (100ms):** Đọc ADC điện áp Pin, cảm biến Gas, cảm biến Nhiệt độ. Đọc tín hiệu Digital từ 3 cảm biến Lửa IR. Áp dụng bộ lọc số (EMA cho gas, Debounce cho lửa IR).
    * **Task_UART_Tx (100ms):** Đóng gói toàn bộ dữ liệu cảm biến (`SensorPacket_t` + CRC8 Checksum) gửi qua UART cho ESP32-S3.

### 3.2. Giao thức UART giữa S3 và WROOM

**Gói tin WROOM → S3 (SensorPacket_t, 14 bytes):**
```c
typedef struct __attribute__((packed)) {
    uint8_t  header;        // 0xAA
    uint8_t  fire_flags;    // Bit 0-2: 3 cảm biến lửa IR (1=có lửa)
    float    gas_ppm;       // Nồng độ gas (đã lọc EMA)
    float    temperature;   // Nhiệt độ (°C)
    float    batt_voltage;  // Điện áp pin (V)
    uint8_t  crc8;          // Checksum
} SensorPacket_t;
```

**Gói tin S3 → WROOM (ActuatorCmd_t, 5 bytes):**
```c
typedef struct __attribute__((packed)) {
    uint8_t header;      // 0xBB
    uint8_t servo_pan;   // Góc xoay ngang vòi phun (0-180, 90 = center)
    uint8_t pump_on;     // 0: Tắt, 1: Bật bơm
    uint8_t buzzer_on;   // 0: Tắt, 1: Bật buzzer
    uint8_t crc8;        // Checksum
} ActuatorCmd_t;
```

## 4. MÁY TRẠNG THÁI (STATE MACHINE)
Được thiết lập trên ESP32-S3, quản lý an toàn cho robot dựa vào trạng thái kết nối micro-ROS:

* **STATE_IDLE (Chờ kết nối):**
    * **Điều kiện:** micro-ROS chưa kết nối hoặc vừa khởi động.
    * **Hành động:** PWM = 0, chờ kết nối.
* **STATE_AUTO (Chế độ Tự động):**
    * **Điều kiện:** micro-ROS connected, nhận `/cmd_vel` từ Nav2.
    * **Hành động:** Xe chạy theo quỹ đạo của bộ điều hướng Nav2.
* **STATE_MANUAL (Chế độ Thủ công):**
    * **Điều kiện:** Operator chuyển mode qua ROS 2 Service/Parameter.
    * **Hành động:** Xe chạy theo lệnh `/cmd_vel` từ `teleop_twist_keyboard`.
* **STATE_EMERGENCY (Dừng Khẩn cấp):**
    * **Điều kiện:** Mất kết nối micro-ROS > 1000ms.
    * **Hành động:** Firmware lập tức đưa toàn bộ PWM về 0 (Phanh cứng 4 bánh), gửi lệnh bật Buzzer liên tục qua WROOM.

## 5. BỘ LỌC DỮ LIỆU CẢM BIẾN (DATA FILTERING)
Được tích hợp trên ESP32-WROOM để tránh hiện tượng báo cháy giả:
* **Lọc Cảm biến Gas (MQ):** Áp dụng thuật toán Trung bình động lũy thừa (EMA) để làm mượt giá trị ADC, tránh gai nhiễu khi bơm nước khởi động. Bỏ qua giá trị trong 60 giây đầu tiên khởi động (thời gian làm nóng cảm biến).
* **Lọc Cảm biến Lửa IR:** Sử dụng Debounce Timer (ngưỡng 200ms). Cảm biến phải báo mức thấp (phát hiện lửa) liên tục trong 200ms thì hệ thống mới xác nhận có lửa thực sự, loại bỏ nhiễu ánh sáng môi trường.

## 6. TIÊU CHÍ NGHIỆM THU FIRMWARE
* **Thời gian dừng khẩn cấp:** Khi mất kết nối micro-ROS, xe phải ngừng di chuyển trong thời gian < 1 giây.
* **Độ chính xác Odometry:** Không xảy ra hiện tượng tràn số (Overflow) của Encoder khi xe chạy tốc độ tối đa; Góc xoay IMU không bị trôi (Drift) quá $2^{\circ}$ trong 10 phút đứng im.
* **Độ ổn định Truyền thông:** Đường truyền UART giữa WROOM và S3 đạt tỷ lệ CRC pass > 99%. Không bị treo (Deadlock) khi có nhiễu điện từ từ động cơ bơm nước.
