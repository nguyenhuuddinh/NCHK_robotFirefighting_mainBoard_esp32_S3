# BẢN MÔ TẢ YÊU CẦU TỔNG THỂ: HỆ THỐNG ROBOT CHỮA CHÁY TỰ HÀNH (AUTONOMOUS FIREFIGHTING ROBOT)

## 1. GIỚI THIỆU MỤC TIÊU
Hệ thống Robot Chữa Cháy Tự Hành là nền tảng robot nâng cao, tập trung vào việc ứng dụng điện toán biên (Edge Computing) và kiến trúc điều khiển phân tán. Mục tiêu cốt lõi của dự án là xây dựng một hệ thống có khả năng tự động hóa việc lập bản đồ (SLAM), điều hướng (Navigation 2), nhận diện hỏa hoạn/nạn nhân bằng AI (YOLO) thông qua mạng diện rộng (WAN 4G/Tailscale), đồng thời phải đảm bảo tính an toàn tuyệt đối với cơ chế ghi đè phần cứng (Hardware Override) trong môi trường thực thi khắt khe.

## 2. KIẾN TRÚC PHẦN CỨNG XE (VEHICLE HARDWARE)
Hệ thống được thiết kế theo kiến trúc 3 tầng xử lý, nhằm triệt tiêu "nút thắt cổ chai" (bottleneck) về băng thông và thời gian thực.

### 2.1. Edge Master (Trạm Xử lý Tại mép): Raspberry Pi
* **Vai trò:** Là "trạm thu phát trung chuyển" (Gateway) giữa phần cứng cấp thấp và Trạm mặt đất (Laptop).
* **Thu thập dữ liệu:** Trực tiếp kết nối và xử lý luồng dữ liệu nặng từ Webcam (nhìn thẳng) và Lidar 2D phục vụ vẽ bản đồ.
* **Truyền thông ngoài (WAN):** Tích hợp module SIM A7680C (LTE 4G). Sử dụng VPN Tailscale để tạo mạng nội bộ ảo, cho phép truyền dữ liệu Node (ROS 2 FastDDS) về Laptop mà không cần dùng chung Wi-Fi vật lý.
* **Giao tiếp nội bộ:** Kết nối cáp USB/Serial trực tiếp xuống module ESP32-S3 (chạy micro-ROS Agent).

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
    * Điều khiển 02 RC Servo (Cụm Pan-Tilt) để xoay vòi phun hướng vào tâm lửa.
    * Điều khiển 01 Relay kích hoạt máy bơm nước.
    * Điều khiển 01 Buzzer phát âm thanh cảnh báo.
* **Nhóm Giao tiếp:** Giao tiếp UART với ESP32-S3 để chuyển tiếp dữ liệu lên Pi. Tích hợp Module NRF24L01+ PA/LNA để sẵn sàng nhận lệnh ghi đè khẩn cấp.

### 2.4. Hệ thống Nguồn (Power Distribution)
* **Kiến trúc:** Cấu trúc Nguồn Kép (Dual Power System) cách ly hoàn toàn để chống sụt áp và nhiễu điện từ (EMI).
    * Nguồn 1: Cấp riêng cho Raspberry Pi, Lidar và mạch logic (5V).
    * Nguồn 2: Cấp cho hệ thống 4 Động cơ, Relay và Máy bơm nước có dòng khởi động lớn (12V/24V tùy cấu hình).
* **An toàn:** Hai hệ thống nguồn bắt buộc phải nối chung Mass (GND) để đảm bảo đồng bộ mức Logic cho các đường truyền tín hiệu (UART/USB).

## 3. KIẾN TRÚC TRẠM KIỂM SOÁT (GROUND STATION & OVERRIDE REMOTE)

### 3.1. Trạm Mặt Đất (Laptop Ground Station)
* **Vai trò:** Là "bộ não AI" xử lý thuật toán nặng, gánh tải cho Raspberry Pi.
* **Chức năng:**
    * Nhận dữ liệu Lidar, chạy thuật toán SLAM (vẽ bản đồ) và Nav2 (quyết định đường đi).
    * Nhận luồng ảnh nén từ Webcam, chạy mô hình mạng nơ-ron YOLO bằng GPU để phát hiện lửa và người trong hiện trường.
    * Mô phỏng và trực quan hóa toàn bộ hệ thống qua RViz và Gazebo.

### 3.2. Tay Cầm Điều Khiển Khẩn Cấp (Hardware Override Remote)
* **Vai trò:** Hệ thống dự phòng mức ưu tiên cao nhất (Failsafe Level 1).
* **Phần cứng:** 01 Vi điều khiển kết hợp cụm Joystick và Module vô tuyến NRF24L01+ PA/LNA.
* **Chức năng:** Bắn tín hiệu RF trực tiếp vào mạch ESP32-WROOM trên xe với tầm xa, bỏ qua hoàn toàn mạng 4G và Raspberry Pi.

## 4. KIẾN TRÚC PHẦN MỀM (SOFTWARE ARCHITECTURE)
Hệ thống cấp thấp trên xe được chạy trên nền tảng FreeRTOS nhằm phân luồng đa tác vụ.

### 4.1. Máy Trạng Thái Động Lực (ESP32-S3 / ESP32-WROOM)
Quản lý luồng ưu tiên điều khiển (Control Hierarchy):
* **STATE_OVERRIDE (Ưu tiên 1):** Khi nhận được gói tin từ NRF24L01+ (Tay cầm), hệ thống phớt lờ mọi lệnh cmd_vel từ ROS 2. Cho phép người dùng lùi xe cứu hộ ngay cả khi rớt mạng 4G.
* **STATE_AUTO_NAV (Ưu tiên 2):** Xe di chuyển theo quỹ đạo của hệ thống Nav2 (Laptop tính toán -> Pi truyền xuống).
* **STATE_EMERGENCY (Ưu tiên 3):** Dừng khẩn cấp, khóa 4 động cơ nếu mất kết nối 4G/Tailscale (> 2000ms) VÀ mất tín hiệu NRF24L01+ (> 1000ms).

## 5. TIÊU CHÍ NGHIỆM THU (ACCEPTANCE CRITERIA)
* **Truyền thông WAN:** Laptop và Raspberry Pi nhận diện được các ROS Nodes của nhau thông qua mạng 4G (Tailscale) với độ trễ (Ping) ổn định để không làm đứt gãy kết nối FastDDS.
* **Động học Skid-Steer:** Dữ liệu Odometry (từ 4 Encoder + IMU6050) xuất ra phải hội tụ, giúp bản đồ SLAM trên Laptop không bị "trượt" (drift) khi xe xoay tại chỗ.
* **Logic An toàn:** Tính năng chuyển đổi (Switch) giữa chế độ tự động (AI/Nav2) sang chế độ thủ công (RF Remote) phải diễn ra tức thì, không có độ trễ treo hệ thống.
* **Nhận diện & Thực thi:** Mô hình YOLO phát hiện được ngọn lửa, Pi tính toán tọa độ tâm ngọn lửa để truyền xuống ESP32-WROOM điều khiển Servo Pan-Tilt khóa mục tiêu chính xác trước khi bật Relay bơm nước.