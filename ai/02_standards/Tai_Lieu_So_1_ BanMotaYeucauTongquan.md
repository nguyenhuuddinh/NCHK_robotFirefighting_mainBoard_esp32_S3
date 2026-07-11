# BẢN MÔ TẢ YÊU CẦU TỔNG THỂ: HỆ THỐNG ROBOT CHỮA CHÁY TỰ HÀNH (AUTONOMOUS FIREFIGHTING ROBOT)

## 1. GIỚI THIỆU MỤC TIÊU
Hệ thống Robot Chữa Cháy Tự Hành là nền tảng robot nâng cao, tập trung vào việc ứng dụng điện toán biên (Edge Computing) và kiến trúc điều khiển phân tán. Mục tiêu cốt lõi của dự án là xây dựng một hệ thống có khả năng tự động hóa việc lập bản đồ (SLAM), điều hướng (Navigation 2), nhận diện hỏa hoạn bằng AI (YOLO) thông qua mạng WiFi nội bộ (LAN), đồng thời phải đảm bảo tính an toàn với cơ chế dừng khẩn cấp khi mất kết nối.

## 2. KIẾN TRÚC PHẦN CỨNG XE (VEHICLE HARDWARE)
Hệ thống được thiết kế theo kiến trúc 3 tầng xử lý, nhằm triệt tiêu "nút thắt cổ chai" (bottleneck) về băng thông và thời gian thực.

### 2.1. Edge Master (Trạm Xử lý Tại mép): Raspberry Pi
* **Vai trò:** Là "trạm thu phát trung chuyển" (Gateway) giữa phần cứng cấp thấp và Trạm mặt đất (Laptop).
* **Thu thập dữ liệu:** Trực tiếp kết nối và xử lý luồng dữ liệu nặng từ Webcam (nhìn thẳng) và Lidar 2D phục vụ vẽ bản đồ.
* **Truyền thông:** Raspberry Pi phát WiFi Access Point (hostapd). Laptop kết nối trực tiếp vào WiFi của Pi — robot hoạt động độc lập, không cần router bên ngoài. ROS 2 DDS Multicast hoạt động bình thường trên mạng nội bộ này.
* **Giao tiếp nội bộ:** Kết nối cáp USB Serial trực tiếp xuống module ESP32-S3 (chạy micro-ROS Agent).

### 2.2. Motion Slave (Module Động lực & Điều hướng): ESP32-S3
* **Vai trò:** Trực tiếp điều khiển hệ thống di chuyển cơ học, yêu cầu phản hồi thời gian thực cực cao (Real-time Kinematics).
* **Hệ thống truyền động:** Điều khiển 4 Động cơ độc lập gắn bánh tròn cao su (Cơ cấu lái trượt - Skid-Steer).
* **Đo lường (Odometry):**
    * Đọc 4 kênh Encoder để tính toán quãng đường và tốc độ thực tế của từng bánh.
    * Đọc IMU MPU6050 để bù trừ sai số trượt (slip) khi xe xoay góc (Yaw).

* **Nhiệm vụ lõi:** Đóng gói toàn bộ dữ liệu Encoder và IMU, gửi thẳng lên Raspberry Pi qua micro-ROS để đáp ứng tần số cập nhật (Update rate) của bộ điều hướng Nav2.

### 2.3. Sensor & Actuator Slave (Module Cảm biến & Chấp hành): ESP32-WROOM
* **Vai trò:** Là "hệ thần kinh ngoại biên", quản lý an toàn, đo lường môi trường và thực thi lệnh chữa cháy.
* **Nhóm Cảm biến An toàn:**
    * 03 Cảm biến Lửa IR gắn phía sau xe (Cảnh báo chống bùng lửa sau lưng).
    * 01 Cảm biến Khí độc (Gas) và 01 Cảm biến Nhiệt độ.
    * Đo lường điện áp Pin qua cầu phân áp (ADC).
* **Nhóm Thực thi (Actuators):**
    * Điều khiển 01 RC Servo (Pan) để xoay vòi phun nước theo phương ngang hướng vào tâm lửa.
    * Điều khiển 01 Relay kích hoạt máy bơm nước.
    * Điều khiển 01 Buzzer phát âm thanh cảnh báo.
* **Nhóm Giao tiếp:** Giao tiếp UART với ESP32-S3 để chuyển tiếp dữ liệu lên Pi.

### 2.4. Hệ thống Nguồn (Power Distribution)
* **Kiến trúc:** Cấu trúc Nguồn Kép (Dual Power System) cách ly hoàn toàn để chống sụt áp và nhiễu điện từ (EMI).
    * Nguồn 1: Cấp riêng cho Raspberry Pi, Lidar và mạch logic (5V).
    * Nguồn 2: Cấp cho hệ thống 4 Động cơ, Relay và Máy bơm nước có dòng khởi động lớn (12V/24V tùy cấu hình).
* **An toàn:** Hai hệ thống nguồn bắt buộc phải nối chung Mass (GND) để đảm bảo đồng bộ mức Logic cho các đường truyền tín hiệu (UART/USB).

## 3. KIẾN TRÚC TRẠM KIỂM SOÁT (GROUND STATION)

### 3.1. Trạm Mặt Đất (Laptop Ground Station)
* **Vai trò:** Là "bộ não AI" xử lý thuật toán nặng, gánh tải cho Raspberry Pi.
* **Chức năng:**
    * Nhận dữ liệu Lidar, chạy thuật toán SLAM (vẽ bản đồ) và Nav2 (quyết định đường đi).
    * Nhận luồng ảnh từ Webcam, chạy mô hình mạng nơ-ron YOLO bằng GPU để phát hiện lửa trong hiện trường.
    * Cung cấp giao diện điều khiển thủ công (Manual/Teleop) qua `teleop_twist_keyboard` hoặc Foxglove Joystick, thay thế hoàn toàn tay cầm RF vật lý.
    * Mô phỏng và trực quan hóa toàn bộ hệ thống qua RViz.

## 4. KIẾN TRÚC PHẦN MỀM (SOFTWARE ARCHITECTURE)
Hệ thống cấp thấp trên xe được chạy trên nền tảng FreeRTOS nhằm phân luồng đa tác vụ.

### 4.1. Máy Trạng Thái Động Lực (ESP32-S3)
Quản lý luồng ưu tiên điều khiển (Control Hierarchy):
* **STATE_AUTO (Chế độ Tự động):** Xe di chuyển theo quỹ đạo của hệ thống Nav2 (Laptop tính toán -> Pi truyền xuống qua `/cmd_vel`).
* **STATE_MANUAL (Chế độ Thủ công):** Người vận hành điều khiển xe từ Laptop qua `teleop_twist_keyboard` hoặc Foxglove Joystick. Lệnh vẫn đi qua ROS 2 `/cmd_vel`, chuyển đổi mode qua ROS 2 Service.
* **STATE_EMERGENCY (Dừng Khẩn cấp):** Dừng khẩn cấp, khóa 4 động cơ nếu mất kết nối micro-ROS (> 1000ms). Kích hoạt Buzzer cảnh báo.

## 5. TIÊU CHÍ NGHIỆM THU (ACCEPTANCE CRITERIA)
* **Truyền thông LAN:** Laptop và Raspberry Pi nhận diện được các ROS Nodes của nhau thông qua WiFi LAN cùng router với độ trễ (Ping) < 10ms.
* **Động học Skid-Steer:** Dữ liệu Odometry (từ 4 Encoder + IMU6050) xuất ra phải hội tụ, giúp bản đồ SLAM trên Laptop không bị "trượt" (drift) khi xe xoay tại chỗ.
* **Logic An toàn:** Khi mất kết nối micro-ROS, xe phải dừng ngay lập tức (< 1 giây), không có độ trễ treo hệ thống.
* **Nhận diện & Thực thi:** Mô hình YOLO phát hiện được ngọn lửa, Laptop tính toán tọa độ tâm ngọn lửa để truyền xuống ESP32-S3, S3 chuyển tiếp qua UART cho ESP32-WROOM điều khiển Servo Pan xoay vòi phun nước khóa mục tiêu chính xác trước khi bật Relay bơm nước.