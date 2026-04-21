# Cấu trúc thư mục cho ESP32-S3 (Motion & micro-ROS Slave)

```text
Firefighting_Main_S3/
├── lib/
│   ├── app/
│   │   ├── RobotMaster.h / .cpp    # Máy trạng thái chính (Quản lý Auto, Override, Emergency).
│   │   ├── TaskManager.h / .cpp    # Phân bổ các task (Task_Motion, Task_MicroROS, Task_Actuator).
│   │   └── ActuatorLogic.h / .cpp  # Xử lý tọa độ tâm lửa để tính góc xoay 2 Servo, đóng mở Relay bơm.
│   │
│   ├── common/
│   │   ├── DataStructs.h           # Struct cấu trúc dữ liệu (Odom, Tốc độ, Lệnh RF từ WROOM gửi sang).
│   │   ├── PinConfig.h             # Mapping chân: 4 Motor, 4 Encoder, I2C, Servo, Relay, UART.
│   │   └── RobotConfig.h           # Các thông số vật lý (Bánh xe, Kp Ki Kd, Góc xoay Servo giới hạn).
│   │
│   ├── driver/
│   │   ├── EncoderDriver.h / .cpp  # Cấu hình ngắt (PCNT) đọc 4 bánh.
│   │   ├── IMUDriver.h / .cpp      # Đọc MPU6050.
│   │   ├── MotorDriver.h / .cpp    # Băm xung PWM điều khiển mạch cầu H.
│   │   ├── ServoDriver.h / .cpp    # Băm xung điều khiển cụm Pan-Tilt vòi phun.
│   │   ├── RelayDriver.h / .cpp    # Đóng ngắt Relay bơm nước.
│   │   └── SlaveComm.h / .cpp      # UART DMA nhận gói tin cảm biến/RF từ WROOM.
│   │
│   └── service/
│       ├── MicroRosComm.h / .cpp   # Quản lý Pub/Sub với Raspberry Pi.
│       ├── Kinematics.h / .cpp     # Động học Lái trượt (Skid-Steer).
│       ├── Odometry.h / .cpp       # Tính toán tọa độ (x, y, yaw).
│       └── PIDController.h / .cpp  # PID tốc độ bánh xe.
│
├── src/
│   └── main.cpp
└── platformio.ini                  # Khai báo lib FreeRTOS, micro-ROS, ESP32Servo.