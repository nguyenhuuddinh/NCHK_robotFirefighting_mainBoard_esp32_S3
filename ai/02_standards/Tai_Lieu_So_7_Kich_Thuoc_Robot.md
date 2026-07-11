# BẢNG KÍCH THƯỚC VẬT LÝ ROBOT CHỮA CHÁY

> Tất cả giá trị đo từ xe thật. Dùng làm cơ sở cho URDF và RobotConfig.h.

## 1. Khung xe (Chassis)
| Thông số | Giá trị (cm) | Giá trị (m) |
|:---|:---:|:---:|
| Chiều dài (CHASSIS_LENGTH) | 25.5 | 0.255 |
| Chiều rộng (CHASSIS_WIDTH) | 15.0 | 0.150 |
| Chiều cao (CHASSIS_HEIGHT) | 5.2 | 0.052 |
| Khoảng cách mặt đất (GROUND_CLEARANCE) | 6.5 | 0.065 |

## 2. Bánh xe (Wheels) — Skid-Steer 4 bánh
| Thông số | Giá trị (cm) | Giá trị (m) |
|:---|:---:|:---:|
| Bán kính bánh (WHEEL_RADIUS) | 3.4 | 0.034 |
| Độ dày bánh (WHEEL_WIDTH) | 2.6 | 0.026 |
| Khoảng cách trục trái-phải (TRACK_WIDTH) | 18.0 | 0.180 |
| Khoảng cách trục trước-sau (WHEELBASE) | 13.4 | 0.134 |

## 3. Vị trí cảm biến (từ tâm base_link, Z từ mặt đất)

> base_link = tâm hình chiếu 4 bánh xe, nằm trên mặt đất (Z=0).
> Gốc đo Z: **đáy khung xe** (ground_clearance = 6.5cm) + giá trị đo.

| Cảm biến | X (cm) | Y (cm) | Z từ đáy xe (cm) | Z từ mặt đất (cm) | Ghi chú |
|:---|:---:|:---:|:---:|:---:|:---|
| **IMU (MPU6050)** | +1.6 | -0.5 | 1.0 | 7.5 | Gắn trên board ESP32, lệch phải |
| **Lidar (RPLidar)** | +9.4 | 0.0 | 11.8 | 18.3 | Trên nóc, ngửa lên quét ngang |
| **Camera (Webcam)** | +12.75 | 0.0 | 6.3 | 12.8 | Đầu xe, nhìn thẳng, nghiêng xuống ~4° |

## 4. Hệ tọa độ ROS 2 (REP-103)
- **X**: Phía trước xe (+), phía sau (-)
- **Y**: Bên trái (+), bên phải (-)
- **Z**: Hướng lên (+)
