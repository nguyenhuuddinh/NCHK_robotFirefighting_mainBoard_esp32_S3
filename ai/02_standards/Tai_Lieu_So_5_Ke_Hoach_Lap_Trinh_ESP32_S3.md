# KẾ HOẠCH LẬP TRÌNH CHI TIẾT: ESP32-S3 (MOTION SLAVE)

Tài liệu này mô tả chi tiết tư duy thiết kế, logic hoạt động và lộ trình triển khai mã nguồn cho bộ vi điều khiển ESP32-S3 trong hệ thống Robot Chữa Cháy Tự Hành.

## 1. TƯ DUY THIẾT KẾ & LOGIC HOẠT ĐỘNG (THE MINDSET)

ESP32-S3 đóng vai trò là "Tiểu não" của hệ thống, chịu trách nhiệm chính về chuyển động thời gian thực (Real-time Kinematics) và giao tiếp với "Đại não" (Raspberry Pi/Laptop) qua micro-ROS.

### 1.1. Kiến trúc Đa luồng (FreeRTOS)
Vì phải xử lý đồng thời giao tiếp ROS 2 (vốn nặng về truyền thông) và tính toán động học (nặng về toán học & thời gian thực), hệ thống bắt buộc phải dùng FreeRTOS với sự phân bổ 2 Core rõ ràng:
- **Core 1 (Motion & Real-time):** Ưu tiên tuyệt đối cho việc đọc Encoder, IMU, tính toán PID vòng kín (Closed-loop) và xuất xung PWM điều khiển động cơ. Điều này đảm bảo Odometry luôn chính xác, không bị gián đoạn bởi các tác vụ mạng.
- **Core 0 (Communication & Logic):** Chuyên trách xử lý giao thức micro-ROS (Pub/Sub) với Raspberry Pi và giao tiếp UART để nhận dữ liệu từ ESP32-WROOM (cảm biến an toàn).

### 1.2. Máy Trạng Thái Cốt Lõi (Core State Machine)
Mọi quyết định di chuyển của ESP32-S3 phụ thuộc vào `RobotMaster`, quản lý 4 trạng thái:
1. **STATE_IDLE:** Chờ kết nối micro-ROS, PWM = 0.
2. **STATE_AUTO:** Chạy theo quỹ đạo từ Nav2. Đọc lệnh từ topic `/cmd_vel`.
3. **STATE_MANUAL:** Chạy theo lệnh teleop từ Laptop. Chuyển mode qua ROS 2 Service.
4. **STATE_EMERGENCY:** Phanh cứng + Buzzer nếu mất kết nối micro-ROS > 1000ms.

### 1.3. Động học Lái Trượt (Skid-Steer Kinematics) & Odometry
- **Inverse Kinematics (Input → Output):**
  - Input: Tốc độ tuyến tính ($V_x$) và tốc độ góc ($\omega$) từ `/cmd_vel`.
  - Output: Vận tốc Setpoint cho 4 bánh xe.
  - $V_{left} = V_x - \omega \cdot L/2$ , $V_{right} = V_x + \omega \cdot L/2$ (L = track width).
  - $\omega_{wheel} = V_{wheel} / R$ (R = bán kính bánh xe).

- **Forward Kinematics (Encoder → Robot velocity):**
  - $V_x = (V_{left} + V_{right}) / 2$
  - $\omega = (V_{right} - V_{left}) / L$

- **Sensor Fusion (Odometry):**
  - Adaptive Complementary Filter cho Yaw:
    - Khi xoay ($|\omega_{gyro}| > 0.1$): $\alpha = 0.995$ (tin gyro).
    - Khi đi thẳng: $\alpha = 0.95$.
    - $\omega_{fused} = \alpha \cdot \omega_{gyro} + (1-\alpha) \cdot \omega_{encoder}$
  - Tích phân vị trí (Euler Forward):
    - $x += V_x \cdot \cos(\theta) \cdot dt$
    - $y += V_x \cdot \sin(\theta) \cdot dt$
    - $\theta += \omega_{fused} \cdot dt$

- **Điều khiển Motor (Feed-Forward + PID):**
  - $PWM = FF(\omega_{target}) + PID(\omega_{target}, \omega_{actual})$
  - $FF(\omega) = sign(\omega) \cdot (PWM_{offset} + |\omega| \cdot K_{FF})$
  - PID: Proportional + Integral (Kd=0 vì encoder rời rạc).
  - Anti-windup: $|I| \leq PWM_{max} / K_i$

---

## 2. KIẾN TRÚC MÃ NGUỒN (CODE STRUCTURE)

Dựa trên cấu trúc thư mục đã quy hoạch, mã nguồn được chia thành 4 lớp (Layers) chính trong thư mục `lib/`:

### Lớp 1: Common (Dữ liệu & Cấu hình)
- **`DataStructs.h`**: Định nghĩa các Struct dùng chung (Ví dụ: `OdometryData_t`, `MotorSpeeds_t`, `SensorPacket_t`, `ActuatorCmd_t`).
- **`PinConfig.h`**: Mapping toàn bộ chân GPIO (Encoder, PWM, I2C, UART, Servo).
- **`RobotConfig.h`**: Chứa các hằng số vật lý (Bán kính bánh xe, khoảng cách trục, thông số PID Kp/Ki/Kd, Ramp, IMU filter alpha).

### Lớp 2: Driver (Giao tiếp Phần cứng cấp thấp)
- **`MotorDriver`**: Cấu hình LEDC để băm xung điều khiển mạch cầu H (L298N).
- **`EncoderDriver`**: Dùng PCNT (Pulse Counter) phần cứng + Glitch Filter 1023 để đọc xung Encoder mà không tốn tài nguyên CPU.
- **`IMUDriver`**: Giao tiếp I2C đọc MPU6050, cache giá trị, Complementary Filter cho pitch/roll, trả gyro_z đã trừ bias.
- **`SlaveComm`**: Cấu hình UART nhận gói tin `SensorPacket_t` từ ESP32-WROOM, gửi `ActuatorCmd_t` (servo pan, bơm, buzzer) xuống WROOM. Parse gói tin + CRC8.

### Lớp 3: Service (Xử lý Thuật toán & Giao thức)
- **`Kinematics`**: Hàm chuyển đổi $(V_x, \omega) \leftrightarrow (V_{FL}, V_{RL}, V_{FR}, V_{RR})$.
- **`Odometry`**: Tính toán tích lũy vị trí $(x, y, \theta)$ bằng Euler Forward tích hợp góc Yaw IMU (Adaptive Complementary Filter).
- **`PIDController`**: Class PID chuẩn hóa cho 4 bánh, có Anti-windup.
- **`MicroRosComm`**: Cấu hình Transport USB Serial, Node, Publisher (`/odom`, `/imu/data`, `/env_status`), Subscriber (`/cmd_vel`, `/fire_target`, `/pump_cmd`), TF broadcaster.

### Lớp 4: App (Logic Ứng dụng & Máy Trạng Thái)
- **`TaskManager`**: Nơi gọi hàm `xTaskCreatePinnedToCore` để khởi tạo các luồng FreeRTOS.
- **`RobotMaster`**: Class chứa biến `currentState` và logic chuyển đổi trạng thái dựa trên trạng thái kết nối micro-ROS.
- **`ActuatorLogic`**: Nhận tọa độ lửa từ YOLO (`/fire_target`), tính góc Servo Pan, đóng gói vào `ActuatorCmd_t` gửi qua UART xuống WROOM.

---

## 3. LỘ TRÌNH PHÁT TRIỂN & CODE (IMPLEMENTATION PLAN)

### Giai đoạn 1: Refactor Code Hiện Tại (2-3 ngày)
- Sửa IMUDriver: gộp đọc I2C vào `update()`, cache giá trị, `getGyroZ()` chỉ trả cache.
- Tạo `RobotMaster` class quản lý state (IDLE/AUTO/MANUAL/EMERGENCY).
- Tạo `TaskManager` tập trung khởi tạo FreeRTOS tasks.
- Tách Web UI ra module riêng (giữ tạm để test, sau bỏ khi có micro-ROS).
- **Verify:** Robot chạy y hệt trước refactor, log PID giống nhau.

### Giai đoạn 2: UART S3 ↔ WROOM (3 ngày)
- Viết `SlaveComm` trên S3: UART2 init + parse `SensorPacket_t` + CRC8.
- Viết firmware WROOM: đọc cảm biến → đóng gói → gửi UART.
- Thêm `Task_UART_Rx` trên S3 Core 0, dùng `xQueueOverwrite` để giữ data mới nhất.
- **Verify:** S3 Serial log hiện data cảm biến từ WROOM, CRC fail < 1%.

### Giai đoạn 3: Cải thiện Odometry (2-3 ngày)
- Triển khai Adaptive Complementary Filter (alpha thay đổi theo tình huống xoay/thẳng).
- Test hình vuông: xe đi 4 cạnh × 1m, kiểm tra sai số vị trí khi về gốc.
- Calibrate `TRACK_WIDTH_M` và `WHEEL_RADIUS_M` bằng thực nghiệm.
- **Verify:** Sai số < 10% sau 4m tổng quãng đường.

### Giai đoạn 4: micro-ROS (5-7 ngày)
- Thêm lib `micro_ros_platformio`, cấu hình transport USB Serial.
- Tạo Publisher: `/odom` (nav_msgs/Odometry), `/imu/data` (sensor_msgs/Imu), `/env_status`.
- Tạo Subscriber: `/cmd_vel` (geometry_msgs/Twist), `/fire_target`, `/pump_cmd`.
- Khi nhận `/fire_target`, `ActuatorLogic` tính góc servo pan và gửi qua UART xuống WROOM.
- Publish TF `odom → base_link` (bắt buộc cho SLAM).
- Enable time sync với micro-ROS Agent.
- **Verify:** `ros2 topic echo /odom` trên Pi hiện data, `ros2 topic pub /cmd_vel` → xe chạy.

### Giai đoạn 5: Tích hợp trên S3 (2 ngày)
- Ghép tất cả: Motion + UART + micro-ROS + Servo + State Machine.
- Test watchdog: rút USB → EMERGENCY < 1s, cắm lại → IDLE.
- Test toàn bộ flow: Laptop pub `/cmd_vel` → S3 chạy → S3 pub `/odom` → Laptop nhận.
- **Verify:** Hệ thống chạy ổn định > 30 phút không crash, không memory leak.
