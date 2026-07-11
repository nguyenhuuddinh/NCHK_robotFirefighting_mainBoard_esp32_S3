# BẢNG QUY HOẠCH CHÂN ESP32-S3 (MOTION & MICRO-ROS SLAVE)

## 1. Nhóm Encoder (Đọc xung tốc độ bánh xe)
| Tên Chức Năng | Chân GPIO | Ghi chú |
| :--- | :---: | :--- |
| **ENC1_A** | GPIO 8 | Kênh A - Động cơ 1 (FL) |
| **ENC1_B** | GPIO 9 | Kênh B - Động cơ 1 (FL) |
| **ENC2_A** | GPIO 10 | Kênh A - Động cơ 2 (RL) |
| **ENC2_B** | GPIO 11 | Kênh B - Động cơ 2 (RL) |
| **ENC3_A** | GPIO 12 | Kênh A - Động cơ 3 (FR) |
| **ENC3_B** | GPIO 13 | Kênh B - Động cơ 3 (FR) |
| **ENC4_A** | GPIO 14 | Kênh A - Động cơ 4 (RR) |
| **ENC4_B** | GPIO 15 | Kênh B - Động cơ 4 (RR) |

## 2. Nhóm Điều khiển Động cơ (Motor Driver - L298N ×2)
| Tên Chức Năng | Chân GPIO | Ghi chú |
| :--- | :---: | :--- |
| **MOT_IN1_FL** | GPIO 4 | Dir Motor 1 (FL) |
| **MOT_IN2_FL** | GPIO 5 | Dir Motor 1 (FL) |
| **MOT_IN3_RL** | GPIO 6 | Dir Motor 2 (RL) |
| **MOT_IN4_RL** | GPIO 7 | Dir Motor 2 (RL) |
| **MOT_ENA_FL** | GPIO 16 | PWM Motor 1 (FL) |
| **MOT_ENB_RL** | GPIO 17 | PWM Motor 2 (RL) |
| **MOT_IN1_FR** | GPIO 18 | Dir Motor 3 (FR) |
| **MOT_IN2_FR** | GPIO 21 | Dir Motor 3 (FR) |
| **MOT_IN3_RR** | GPIO 39 | Dir Motor 4 (RR) |
| **MOT_IN4_RR** | GPIO 40 | Dir Motor 4 (RR) |
| **MOT_ENA_FR** | GPIO 41 | PWM Motor 3 (FR) |
| **MOT_ENB_RR** | GPIO 42 | PWM Motor 4 (RR) |

## 3. Nhóm Giao tiếp Ngoại vi
| Tên Chức Năng | Chân GPIO | Ghi chú |
| :--- | :---: | :--- |
| **I2C_SDA** | GPIO 2 | I2C Data (MPU6050) |
| **I2C_SCL** | GPIO 1 | I2C Clock (MPU6050) |
| **UART2_TX** | GPIO 19 | TX → WROOM RX |
| **UART2_RX** | GPIO 20 | RX ← WROOM TX |

## 4. Tổng hợp sử dụng GPIO

| Nhóm | Số chân | GPIO |
| :--- | :---: | :--- |
| Encoder | 8 | 8-15 |
| Motor DIR | 8 | 4, 5, 6, 7, 18, 21, 39, 40 |
| Motor PWM | 4 | 16, 17, 41, 42 |
| I2C (IMU) | 2 | 1, 2 |
| UART (WROOM) | 2 | 19, 20 |
| **Tổng** | **24** | |

## 5. Chân đặc biệt (Không sử dụng)
| GPIO | Lý do |
| :---: | :--- |
| 0 | Strapping pin (boot mode) |
| 43, 44 | UART0 mặc định (Serial Monitor / micro-ROS USB) |
| 45 | Strapping pin (VDD_SPI voltage) |
| 3, 46, 47, 48 | Còn trống — dự phòng tương lai (Servo đã chuyển sang WROOM) |
| 22-34 | Không tồn tại trên ESP32-S3 |
| 26-32 | Dùng cho PSRAM/Flash (tùy module) |
