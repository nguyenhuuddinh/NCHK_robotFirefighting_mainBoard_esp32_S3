# BẢN MÔ TẢ GIAO THỨC & GIAO DIỆN: TRẠM KIỂM SOÁT MẶT ĐẤT

## 1. TỔNG QUAN HỆ THỐNG GIAO TIẾP (COMMUNICATION OVERVIEW)
Hệ thống sử dụng một luồng giao tiếp duy nhất qua WiFi LAN:
* **Luồng Dữ liệu Chính (Data Link):** Giao thức ROS 2 (DDS) chạy trên mạng WiFi nội bộ do Raspberry Pi phát (Access Point). Laptop kết nối trực tiếp vào WiFi của Pi, sử dụng Simple Discovery Protocol (Multicast). Phục vụ AI, vẽ bản đồ, điều khiển tự động và giám sát qua Web Dashboard.

## 2. QUY HOẠCH GIAO THỨC ROS 2 (ROS 2 TOPICS DICTIONARY)
Để Laptop (Trạm mặt đất) và Raspberry Pi (Trạm trung chuyển) hiểu nhau, hệ thống sử dụng các Topic theo chuẩn thông điệp của ROS 2.

### 2.1. Luồng dữ liệu: Robot → Trạm Mặt Đất (Telemetry & Sensors)

| Tên Topic | Kiểu Dữ Liệu (Message Type) | Nguồn phát | Mục đích sử dụng tại Laptop |
| :--- | :--- | :--- | :--- |
| `/scan` | `sensor_msgs/LaserScan` | Lidar (Pi) | Chạy thuật toán SLAM vẽ bản đồ 2D. |
| `/image_raw` | `sensor_msgs/Image` | Webcam (Pi) | Chạy mô hình mạng nơ-ron YOLO phát hiện lửa. |
| `/odom` | `nav_msgs/Odometry` | ESP32-S3 | Tính toán hệ quy chiếu xe trong không gian (TF Tree). |
| `/imu/data` | `sensor_msgs/Imu` | ESP32-S3 | Bù trừ độ trễ/trượt của Odometry. |
| `/env_status` | `std_msgs/String` (JSON) | ESP32-S3 (từ WROOM) | Hiển thị nồng độ Gas, Cảnh báo cháy sau lưng, Pin. |

### 2.2. Luồng dữ liệu: Trạm Mặt Đất → Robot (Commands)

| Tên Topic | Kiểu Dữ Liệu (Message Type) | Nguồn phát | Mục đích sử dụng tại Robot |
| :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 hoặc Teleop (Laptop) | Gửi Vận tốc tuyến tính ($v_x$) và Vận tốc góc ($\omega$) cho S3. |
| `/fire_target` | `geometry_msgs/Point` | YOLO Node (Laptop) | Tọa độ tâm lửa chuẩn hóa [-1,1] để S3 chuyển tiếp qua UART cho WROOM điều khiển Servo Pan vòi phun. |
| `/pump_cmd` | `std_msgs/Bool` | Logic Node (Laptop) | Ra lệnh cho S3 chuyển tiếp xuống WROOM đóng/ngắt Relay bơm nước. |

### 2.3. TF Tree (Transform Tree)

```
map → odom → base_link → laser_frame
                       → camera_frame
                       → imu_frame
```

* `odom → base_link`: ESP32-S3 publish (qua micro-ROS).
* `base_link → laser_frame`: Static transform, cấu hình trong URDF.
* `base_link → camera_frame`: Static transform, cấu hình trong URDF.
* `map → odom`: SLAM node trên Laptop publish.

## 3. GIAO DIỆN TRẠM MẶT ĐẤT (GROUND STATION DASHBOARD)
Sử dụng **Web Dashboard** tùy chỉnh (HTML/CSS/JS + rosbridge) hoặc RViz 2 cài đặt trên Laptop. Laptop kết nối WiFi AP của Pi, mở trình duyệt `http://10.0.0.1:8080` để giám sát.

### 3.1. Các Module Hiển Thị Chính (UI Panels)
* **Map & Navigation View (Trung tâm):** Hiển thị bản đồ Occupancy Grid (Trắng/Đen/Xám) do SLAM tạo ra theo thời gian thực. Cung cấp công cụ "2D Nav Goal" để người dùng click chọn điểm đích cho xe tự chạy tới.
* **AI Vision View:** Hiển thị luồng Video từ camera. Khi YOLO phát hiện ngọn lửa, Bounding Box (khung viền đỏ) sẽ tự động vẽ đè lên luồng video kèm theo tỷ lệ tự tin (Confidence Score).
* **Telemetry Diagnostics:** Hiển thị điện áp Pin, nồng độ khí Gas và nhiệt độ môi trường xung quanh xe.
* **Teleop Panel:** Joystick ảo hoặc `teleop_twist_keyboard` để điều khiển xe thủ công, thay thế hoàn toàn tay cầm RF vật lý.

## 4. TIÊU CHÍ NGHIỆM THU GIAO DIỆN & TRUYỀN THÔNG
* **Băng thông Hình ảnh:** Qua WiFi AP của Pi, camera duy trì luồng video `/image_raw/compressed` ở mức tối thiểu 10 FPS về Web Dashboard.
* **Phản ứng AI (YOLO latency):** Thời gian từ lúc Camera trên Pi thấy ngọn lửa → Truyền WiFi → Laptop YOLO nhận diện → Phản hồi tọa độ về Robot phải dưới 500ms.
* **Phản hồi điều khiển:** Khi dùng Teleop (Web Dashboard hoặc keyboard), xe phải phản hồi lệnh trong vòng 50ms.
* **Recovery (Khôi phục kết nối):** Khi Laptop mất WiFi tạm thời, hệ thống ROS 2 tự động re-discovery trong vòng < 3 giây khi kết nối lại.
