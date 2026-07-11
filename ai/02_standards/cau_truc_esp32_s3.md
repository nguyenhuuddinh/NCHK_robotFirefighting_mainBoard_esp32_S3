# Cấu trúc thư mục cho ESP32-S3 (Motion & micro-ROS Slave)

```text
Firefighting_Main_S3/
├── lib/
│   ├── app/
│   │   ├── RobotMaster.h / .cpp    # Máy trạng thái chính (Idle, Auto, Manual, Emergency).
│   │   ├── TaskManager.h / .cpp    # Phân bổ các task FreeRTOS (Task_Motion, Task_MicroROS, Task_UART_Rx).
│   │   └── ActuatorLogic.h / .cpp  # Xử lý tọa độ tâm lửa từ YOLO để tính góc xoay Servo, ra lệnh bơm.
│   │
│   ├── common/
│   │   ├── DataStructs.h           # Struct cấu trúc dữ liệu (OdometryData_t, MotorSpeeds_t, SensorPacket_t, ActuatorCmd_t).
│   │   ├── PinConfig.h             # Mapping chân: 4 Motor, 4 Encoder, I2C, Servo, UART.
│   │   └── RobotConfig.h           # Các thông số vật lý (Bánh xe, Kp Ki Kd, Ramp, IMU alpha).
│   │
│   ├── driver/
│   │   ├── EncoderDriver.h / .cpp  # Cấu hình PCNT (Pulse Counter) + Glitch Filter đọc 4 bánh.
│   │   ├── IMUDriver.h / .cpp      # Đọc MPU6050, cache giá trị, Complementary Filter (pitch/roll).
│   │   ├── MotorDriver.h / .cpp    # Băm xung PWM điều khiển mạch cầu H (L298N).
│   │   └── SlaveComm.h / .cpp      # UART nhận SensorPacket_t từ WROOM, gửi ActuatorCmd_t (servo pan, bơm, buzzer) xuống WROOM.
│   │
│   └── service/
│       ├── MicroRosComm.h / .cpp   # Quản lý Pub/Sub micro-ROS + TF broadcaster.
│       ├── Kinematics.h / .cpp     # Động học Lái trượt (Skid-Steer) — Inverse & Forward.
│       ├── Odometry.h / .cpp       # Tính toán tọa độ (x, y, yaw) — Adaptive Complementary Filter.
│       └── PIDController.h / .cpp  # PID tốc độ bánh xe + Anti-windup.
│
├── src/
│   └── main.cpp                    # Entry point: setup() + loop(). Gọi TaskManager.init().
└── platformio.ini                  # Khai báo lib FreeRTOS, micro-ROS, ESP32Servo, ESP32Encoder.
```

## So sánh với bản gốc

| Bỏ | Thêm/Sửa |
| :--- | :--- |
| `RelayDriver` (chuyển sang WROOM) | `SlaveComm` — giao tiếp UART với WROOM (parse + CRC8) |
| — | `RobotMaster` — State Machine 4 trạng thái (bỏ RF Override) |
| — | `TaskManager` — tập trung quản lý FreeRTOS tasks |
| — | `MicroRosComm` — thay thế Web Server để giao tiếp với ROS 2 |