# BẢN MÔ TẢ KIẾN TRÚC PHẦN MỀM: RASPBERRY PI MASTER (ROS 2 GATEWAY)

## 1. MÔI TRƯỜNG PHÁT TRIỂN & NỀN TẢNG
* **Hệ điều hành:** Ubuntu Server 22.04 LTS (Tối ưu cho Raspberry Pi).
* **Middleware:** ROS 2 (Humble Hawksbill).
* **Mạng:** Raspberry Pi phát WiFi Access Point (hostapd + dnsmasq). Laptop kết nối trực tiếp vào WiFi của Pi — không cần router bên ngoài, robot hoạt động độc lập ở mọi nơi. ROS 2 DDS Simple Discovery (Multicast) hoạt động bình thường trên mạng nội bộ này.
* **Quản lý giao tiếp:** micro-ROS Agent (Cầu nối Serial-to-ROS2 cho ESP32-S3).

## 2. PHÂN TẦNG KIẾN TRÚC PHẦN MỀM (SOFTWARE STACK)
Hệ thống trên Pi được tổ chức thành các tầng chức năng nhằm đảm bảo tính cô lập và dễ bảo trì:
* **Lớp Driver & Kết nối (Hardware Interface Layer):** Điều khiển trực tiếp ngoại vi cấp cao (Lidar, Webcam) và quản lý kết nối vật lý với ESP32-S3.
* **Lớp Trung gian (Middleware Layer):** Đóng gói gói tin ROS 2 và quản lý danh mục các Topic.
* **Lớp Giám sát & An toàn (Supervisory Layer):** Kiểm tra trạng thái kết nối với Laptop (Ground Station) và thực hiện các lệnh dừng khẩn cấp tại chỗ.

## 3. CẤU TRÚC CÁC NODE ROS 2 CHÍNH
Thay vì xử lý thuật toán nặng, Raspberry Pi tập trung vào việc quản lý các Node thu thập và chuyển tiếp:

| Tên Node | Vai trò chính | Dữ liệu Publish (Output) |
| :--- | :--- | :--- |
| **rplidar_node** | Đọc dữ liệu quét laser từ Lidar. | `/scan` (Dữ liệu khoảng cách 2D). |
| **usb_cam_node** | Thu thập luồng ảnh từ Webcam phía trước. | `/image_raw` (Ảnh gốc). |
| **uros_agent_node** | Cầu nối nhận dữ liệu từ ESP32-S3. | `/odom`, `/imu/data`, `/env_status`. |
| **safety_watchdog** | Giám sát nhịp tim (Heartbeat) từ Laptop. | `/cmd_vel` (Lệnh dừng nếu mất kết nối). |
| **rosbridge_server** | WebSocket bridge cho Web Dashboard trên Laptop. | Chuyển tiếp ROS 2 topics qua WebSocket (port 9090). |

## 4. CẤU HÌNH MẠNG WIFI (Pi là Access Point)
Raspberry Pi phát WiFi AP để Laptop kết nối trực tiếp — robot hoàn toàn độc lập, không phụ thuộc router:
* **Cấu hình AP:** Dùng `hostapd` + `dnsmasq` trên Pi. SSID: `FireRobot_AP`, Password tùy chọn. Pi có IP cố định (ví dụ: `10.0.0.1`), Laptop được cấp IP tự động qua DHCP.
* **ROS 2 Discovery:** Simple Discovery (Multicast) mặc định. Cả Pi và Laptop đặt cùng `ROS_DOMAIN_ID=0`.
* **Ảnh Camera:** Truyền `/image_raw/compressed` (JPEG nén) để tiết kiệm băng thông WiFi. Dùng `image_transport` với plugin `compressed`.
* **DDS QoS (Quality of Service):**
    * *Sensors (Lidar/Camera):* Best Effort (Ưu tiên tốc độ, chấp nhận mất gói).
    * *Control (Lệnh di chuyển):* Reliable (Bắt buộc nhận đủ, đảm bảo an toàn).

## 5. WEB DASHBOARD (Giao diện Giám sát trên Laptop)
Một ứng dụng web đơn giản chạy trên Pi (hoặc Laptop), cho phép người vận hành giám sát robot qua trình duyệt mà không cần cài ROS 2 trên máy xem.

### 5.1. Kiến trúc Web Dashboard
* **Backend:** `rosbridge_websocket` (ROS 2 package) chạy trên Pi, mở WebSocket tại port 9090.
* **Frontend:** Trang web HTML/CSS/JS (host trên Pi hoặc Laptop), kết nối WebSocket qua thư viện `roslibjs`.
* **Truy cập:** Laptop mở trình duyệt → `http://10.0.0.1:8080` (hoặc IP của Pi).

### 5.2. Các Module Hiển Thị
* **Camera Feed:** Hiển thị luồng video từ `/image_raw/compressed` (dùng `ros2djs` hoặc MJPEG stream). Khi YOLO phát hiện lửa/người, hiển thị cảnh báo trên video.
* **Sensor Panel:** Hiển thị nhiệt độ, nồng độ Gas, điện áp Pin từ topic `/env_status` (cập nhật realtime).
* **Alert Panel:** Cảnh báo khi phát hiện lửa (từ cảm biến IR hoặc YOLO), cảnh báo Gas vượt ngưỡng, cảnh báo Pin yếu.
* **Robot Status:** Trạng thái hiện tại (IDLE/AUTO/MANUAL/EMERGENCY), tọa độ Odometry (x, y, yaw).
* **Teleop Control:** Nút điều khiển cơ bản (Tiến/Lùi/Trái/Phải/Dừng) gửi `/cmd_vel` qua WebSocket — dự phòng khi không dùng `teleop_twist_keyboard`.

## 6. LOGIC AN TOÀN & DỰ PHÒNG (FAIL-SAFE LOGIC)
Phần mềm trên Pi đóng vai trò là chốt chặn an toàn cuối cùng trước khi lệnh xuống động cơ:
* **Mất kết nối Laptop (WiFi Fail):** Nếu Node `safety_watchdog` không nhận được dữ liệu từ Topic `/cmd_vel` (do Laptop gửi về) quá 500ms, Pi sẽ tự động gửi lệnh vận tốc bằng 0 xuống ESP32-S3.
* **Quản lý tài nguyên:** Sử dụng Cgroups để giới hạn tài nguyên CPU cho Node xử lý ảnh, ưu tiên băng thông tuyệt đối cho dữ liệu Lidar và micro-ROS Agent để tránh treo hệ thống dẫn đường.

## 7. TIÊU CHÍ NGHIỆM THU PHẦN MỀM MASTER
* **Độ trễ truyền tin:** Lệnh điều khiển từ Laptop qua WiFi AP đến được ESP32-S3 với độ trễ phản hồi tổng thể < 30ms.
* **Tính ổn định:** Laptop tự động kết nối lại WiFi AP của Pi khi mất kết nối tạm thời, các ROS Node re-discovery trong < 3 giây.
* **Hiệu suất:** Tải CPU của Raspberry Pi duy trì mức < 70% khi đang chạy AP + Lidar + Camera + rosbridge + micro-ROS Agent.
* **Web Dashboard:** Trang web load thành công, hiển thị camera feed ≥ 10 FPS, sensor data cập nhật mỗi 1 giây.
