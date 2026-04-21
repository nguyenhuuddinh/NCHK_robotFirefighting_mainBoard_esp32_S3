# BẢN MÔ TẢ KIẾN TRÚC FIRMWARE: CỤM SLAVE ĐIỀU KHIỂN ĐỘNG LỰC VÀ CẢM BIẾN (ESP32-S3 & ESP32-WROOM)

## 1. TỔNG QUAN KIẾN TRÚC PHÂN TÁN (DISTRIBUTED FIRMWARE ARCHITECTURE)
Hệ thống cấp thấp (Low-level Control) được phân tách thành 2 vi điều khiển chạy độc lập trên nền tảng FreeRTOS.
* **ESP32-S3 (Motion Slave):** Đóng vai trò là một micro-ROS Node chính thức, giao tiếp trực tiếp với Raspberry Pi. Tập trung toàn bộ tài nguyên tính toán cho hệ chuyển động.
* **ESP32-WROOM (Sensor & Safety Slave):** Đóng vai trò là Trạm thu thập cảm biến và chốt chặn an toàn (Hardware Override), giao tiếp với ESP32-S3 thông qua UART.

## 2. KIẾN TRÚC FIRMWARE: ESP32-S3 (MOTION SLAVE)
Sử dụng sức mạnh Dual-core của lõi Xtensa LX7 để duy trì tần số cập nhật Odometry ở mức > 50Hz.

### 2.1. Phân bổ Task (FreeRTOS Core Allocation)
* 🔴 **Core 1 (Motion Core - Ưu tiên Cao):**
    * **Task_Encoder (10ms):** Đọc tích lũy 4 bộ Encoder phần cứng.
    * **Task_IMU (10ms):** Đọc MPU6050 qua I2C, áp dụng bộ lọc Kalman/Mahony để lấy góc Yaw với độ trễ thấp.
    * **Task_Kinematics (20ms):** Tính toán Động học Lái trượt (Skid-Steer Kinematics) để chuyển đổi từ $(V_x, \omega)$ của ROS 2 sang tốc độ 4 bánh, chạy PID và xuất Hardware PWM.
* 🔵 **Core 0 (Comms Core - Ưu tiên Trung bình):**
    * **Task_microROS (10ms):** Chạy Executor của micro-ROS, Publish Topic `/odom`, `/imu` lên Pi và Subscribe Topic `/cmd_vel` từ Pi.
    * **Task_UART_Listen (20ms):** Nhận gói tin trạng thái từ WROOM (Dữ liệu lửa, gas, pin) và đẩy vào hàng đợi (Queue) để micro-ROS Publish tiếp lên Pi.

### 2.2. Xử lý Động học Lái trượt (Skid-Steer)
Khác với xe Mecanum cũ, xe 4 bánh cao su khi xoay tại chỗ sẽ sinh ra độ trượt (slip) rất lớn. Firmware phải kết hợp **Vận tốc tính từ Encoder** và **Gia tốc góc Yaw từ IMU** (Sensor Fusion) để tính ra tọa độ Odometry $(x, y, 	heta)$ thực tế của xe, đảm bảo bản đồ SLAM trên Laptop không bị xoay lệch.

## 3. KIẾN TRÚC FIRMWARE: ESP32-WROOM (SENSOR & SAFETY SLAVE)
Firmware này hoạt động theo cơ chế Hướng sự kiện (Event-Driven) để ưu tiên tuyệt đối cho sóng vô tuyến khẩn cấp.

### 3.1. Phân bổ Task (FreeRTOS Core Allocation)
* 🔴 **Core 1 (Safety Core - Ưu tiên Tối đa):**
    * **Task_RF_Listen (5ms):** Liên tục lắng nghe gói tin từ Module NRF24L01+ PA/LNA. Nếu có tín hiệu từ Tay cầm, ngay lập tức kích hoạt cờ FLAG_OVERRIDE.
    * **Task_Actuator (20ms):** Điều khiển 2 Servo Pan-Tilt hướng vòi phun và kích Relay bơm nước dựa trên lệnh nhận được.
* 🔵 **Core 0 (Sensor Core - Ưu tiên Trung bình):**
    * **Task_Sensor_Read (50ms):** Đọc ADC điện áp Pin, cảm biến Gas, cảm biến Nhiệt độ. Đọc tín hiệu Digital từ 3 cảm biến Lửa IR.
    * **Task_Telemetry_TX (50ms):** Đóng gói toàn bộ dữ liệu cảm biến (Struct + CRC8 Checksum) gửi qua UART cho ESP32-S3.

## 4. MÁY TRẠNG THÁI VÀ LOGIC GHI ĐÈ (HARDWARE OVERRIDE LOGIC)
Đây là "linh hồn" bảo vệ an toàn cho robot, được thiết lập trên ESP32-S3 dựa vào tín hiệu nó nhận được từ cả Pi và WROOM:
* **STATE_OVERRIDE (Cấp cao nhất):**
    * **Điều kiện Kích hoạt:** ESP32-S3 nhận được tín hiệu cờ FLAG_OVERRIDE từ WROOM (nghĩa là người dùng đang gạt Joystick trên tay cầm RF).
    * **Hành động:** S3 tự động cắt đứt (ignore) mọi lệnh `/cmd_vel` đến từ Raspberry Pi. Lấy dữ liệu vận tốc từ gói tin RF để điều khiển xe lùi lại hoặc chuyển hướng.
* **STATE_AUTO (Cấp trung bình):**
    * **Điều kiện Kích hoạt:** Không có sóng RF, có kết nối micro-ROS ổn định với Pi.
    * **Hành động:** Xe chạy theo quỹ đạo của bộ điều hướng Nav2.
* **STATE_EMERGENCY (Trạng thái Dừng/Khóa):**
    * **Điều kiện Kích hoạt:** Mất sóng RF (hoặc tay cầm tắt) VÀ Ping micro-ROS bị timeout (> 1000ms).
    * **Hành động:** Firmware lập tức đưa toàn bộ PWM về 0 (Phanh điện từ cứng 4 bánh), hú còi Buzzer liên tục để báo hiệu mất kiểm soát mạng.

## 5. BỘ LỌC DỮ LIỆU CẢM BIẾN (DATA FILTERING)
Để tránh hiện tượng báo cháy giả, Lớp Dịch vụ (Service Layer) trên ESP32-WROOM tích hợp các thuật toán lọc số:
* **Lọc Cảm biến Gas (MQ):** Áp dụng thuật toán Trung bình động lũy thừa (EMA) để làm mượt giá trị ADC, tránh gai nhiễu khi bơm nước khởi động. Bỏ qua giá trị trong 60 giây đầu tiên khởi động (thời gian làm nóng cảm biến).
* **Lọc Cảm biến Lửa IR:** Sử dụng Debounce Timer (ngưỡng 200ms). Cảm biến phải báo mức thấp (phát hiện lửa) liên tục trong 200ms thì hệ thống mới xác nhận có lửa thực sự, loại bỏ nhiễu ánh sáng môi trường.

## 6. TIÊU CHÍ NGHIỆM THU FIRMWARE
* **Thời gian chuyển đổi (Switching Latency):** Khi người dùng nhấn nút trên Tay cầm RF, xe phải ngừng lệnh tự động và chạy theo tay cầm trong thời gian $< 50ms$.
* **Độ chính xác Odometry:** Không xảy ra hiện tượng tràn số (Overflow) của Encoder khi xe chạy tốc độ tối đa; Góc xoay IMU không bị trôi (Drift) quá $2^{\circ}$ trong 10 phút đứng im.
* **Độ ổn định Truyền thông:** Đường truyền UART DMA giữa WROOM và S3 không bị treo (Deadlock) khi có nhiễu điện từ từ động cơ bơm nước.
