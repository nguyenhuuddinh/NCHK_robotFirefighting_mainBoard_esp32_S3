# BẢN MÔ TẢ GIAO THỨC & GIAO DIỆN: TRẠM KIỂM SOÁT MẶT ĐẤT VÀ TAY CẦM KHẨN CẤP

## 1. TỔNG QUAN HỆ THỐNG GIAO TIẾP (COMMUNICATION OVERVIEW)
Hệ thống sử dụng hai luồng giao tiếp hoàn toàn độc lập để đảm bảo tính dự phòng (Redundancy):
* **Luồng Dữ liệu Chính (Data Link):** Giao thức ROS 2 (DDS) chạy trên nền tảng mạng riêng ảo Tailscale (WAN 4G), phục vụ AI, vẽ bản đồ và điều khiển tự động.
* **Luồng Điều khiển Khẩn cấp (Override Link):** Giao thức vô tuyến 2.4GHz sử dụng NRF24L01+ PA/LNA, bỏ qua mạng Internet, kết nối thẳng từ tay cầm vào hệ thống chấp hành cấp thấp.

## 2. QUY HOẠCH GIAO THỨC ROS 2 (ROS 2 TOPICS DICTIONARY)
Để Laptop (Trạm mặt đất) và Raspberry Pi (Trạm trung chuyển) hiểu nhau, hệ thống sử dụng các Topic theo chuẩn thông điệp của ROS 2.

### 2.1. Luồng dữ liệu: Robot $ightarrow$ Trạm Mặt Đất (Telemetry & Sensors)

| Tên Topic | Kiểu Dữ Liệu (Message Type) | Nguồn phát | Mục đích sử dụng tại Laptop |
| :--- | :--- | :--- | :--- |
| `/scan` | `sensor_msgs/LaserScan` | Lidar (Pi) | Chạy thuật toán SLAM vẽ bản đồ 2D. |
| `/image_raw/compressed` | `sensor_msgs/CompressedImage` | Webcam (Pi) | Chạy mô hình mạng nơ-ron YOLO phát hiện lửa. |
| `/odom` | `nav_msgs/Odometry` | ESP32-S3 | Tính toán hệ quy chiếu xe trong không gian (TF Tree). |
| `/imu/data` | `sensor_msgs/Imu` | ESP32-S3 | Bù trừ độ trễ/trượt của Odometry. |
| `/environment_status` | `custom_msgs/EnvStatus` | ESP32-WROOM | Hiển thị nồng độ Gas, Cảnh báo cháy sau lưng, Pin. |

### 2.2. Luồng dữ liệu: Trạm Mặt Đất $ightarrow$ Robot (Commands)

| Tên Topic | Kiểu Dữ Liệu (Message Type) | Nguồn phát | Mục đích sử dụng tại Robot |
| :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 (Laptop) | Gửi Vận tốc tuyến tính ($v_x$) và Vận tốc góc ($\omega$) cho S3. |
| `/fire_target` | `geometry_msgs/Point` | YOLO (Laptop) | Tọa độ tâm lửa để S3 tính toán góc xoay Servo vòi phun. |
| `/pump_cmd` | `std_msgs/Bool` | Logic (Laptop) | Ra lệnh cho WROOM đóng/ngắt Relay bơm nước. |

## 3. GIAO DIỆN TRẠM MẶT ĐẤT (GROUND STATION DASHBOARD)
Sử dụng phần mềm Foxglove Studio (hoặc RViz 2 kết hợp rqt) cài đặt trên Laptop, cung cấp giao diện quản lý tập trung cho người điều hành (Operator).

### 3.1. Các Module Hiển Thị Chính (UI Panels)
* **Map & Navigation View (Trung tâm):** Hiển thị bản đồ Occupancy Grid (Trắng/Đen/Xám) do SLAM tạo ra theo thời gian thực. Cung cấp công cụ "2D Nav Goal" để người dùng click chọn điểm đích cho xe tự chạy tới.
* **AI Vision View:** Hiển thị luồng Video đã được giải nén. Khi YOLO phát hiện ngọn lửa hoặc người bị nạn, Bounding Box (khung viền đỏ/xanh) sẽ tự động vẽ đè lên luồng video kèm theo tỷ lệ tự tin (Confidence Score).
* **Telemetry Diagnostics:** Biểu đồ đường (Line Chart) theo dõi điện áp Pin, nồng độ khí Gas (MQ-7) và đồ thị nhiệt độ môi trường xung quanh xe.

## 4. GIAO THỨC TAY CẦM KHẨN CẤP (RF OVERRIDE CONTROLLER)
Đây là hệ thống Cứu hộ độc lập (Failsafe Level 1).

### 4.1. Cấu trúc Phần cứng Tay cầm
* **MCU:** ESP32 hoặc STM32.
* **Module RF:** NRF24L01+ PA/LNA (Kèm mạch nguồn phụ AMS1117 3.3V và tụ lọc 100uF).
* **Nhập liệu:** 02 Cụm Joystick (Điều khiển xe và Xoay vòi phun), 02 Nút nhấn (Kích hoạt bơm nước, Bấm còi Buzzer).

### 4.2. Cấu trúc Gói tin RF (Payload Structure)
Sử dụng kiểu cấu trúc dữ liệu tĩnh (Struct) được tối ưu hóa giới hạn trong 32 Bytes của NRF24L01 để đảm bảo tốc độ truyền cực nhanh:

```c
typedef struct {
    uint8_t header;       // Byte nhận diện (vd: 0xAA)
    int16_t joy_y;        // Vận tốc tới lùi (Tiến/Lùi)
    int16_t joy_x;        // Vận tốc góc (Xoay trái/phải)
    uint8_t servo_pan;    // Góc quay ngang Camera/Vòi (0-180)
    uint8_t servo_tilt;   // Góc quay dọc Camera/Vòi (0-180)
    uint8_t pump_state;   // Trạng thái Bơm (0: Tắt, 1: Bật)
    uint8_t crc8;         // Mã kiểm tra lỗi dữ liệu
} OverridePacket;
```

## 5. TIÊU CHÍ NGHIỆM THU GIAO DIỆN & TRUYỀN THÔNG
* **Băng thông Hình ảnh:** Module 4G (A7680C) duy trì được luồng video nén chuẩn H.264/JPEG ở mức tối thiểu 10-15 FPS về Trạm mặt đất mà không làm sập mạng DDS.
* **Phản ứng AI (YOLO latency):** Thời gian từ lúc Camera trên Pi thấy ngọn lửa $ightarrow$ Nén $ightarrow$ Truyền 4G $ightarrow$ Laptop giải nén $ightarrow$ YOLO nhận diện $ightarrow$ Phản hồi tọa độ về Robot phải dưới $500ms$.
* **Độ tin cậy RF:** Ở môi trường trống (Line of Sight), tay cầm NRF24L01+ duy trì kết nối ổn định ở khoảng cách $500m$. Khi gạt Joystick, xe phải phản hồi lệnh trong vòng 50ms.
* **Recovery (Khôi phục kết nối):** Khi xe đi vào vùng "lõm" sóng 4G và mất mạng tạm thời, hệ thống ROS 2 trên Laptop và Pi tự động "thấy" lại nhau (re-discovery) trong vòng $< 3$ giây khi có sóng trở lại.
