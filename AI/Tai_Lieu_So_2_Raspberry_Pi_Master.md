# BẢN MÔ TẢ KIẾN TRÚC PHẦN MỀM: RASPBERRY PI MASTER (ROS 2 GATEWAY)

## 1. MÔI TRƯỜNG PHÁT TRIỂN & NỀN TẢNG
* **Hệ điều hành:** Ubuntu Server 22.04 LTS (Tối ưu cho Raspberry Pi).
* **Middleware:** ROS 2 (Humble Hawksbill).
* **Mạng ảo:** Tailscale VPN (Tạo kênh truyền Unicast bảo mật qua 4G).
* **Quản lý giao tiếp:** micro-ROS Agent (Cầu nối Serial-to-ROS2 cho các Slave ESP32).

## 2. PHÂN TẦNG KIẾN TRÚC PHẦN MỀM (SOFTWARE STACK)
Hệ thống trên Pi được tổ chức thành các tầng chức năng nhằm đảm bảo tính cô lập và dễ bảo trì:
* **Lớp Driver & Kết nối (Hardware Interface Layer):** Điều khiển trực tiếp ngoại vi cấp cao (Lidar, Webcam) và quản lý kết nối vật lý với Slave.
* **Lớp Trung gian (Middleware Layer):** Xử lý nén dữ liệu hình ảnh, đóng gói gói tin ROS 2 và quản lý danh mục các Topic.
* **Lớp Giám sát & An toàn (Supervisory Layer):** Kiểm tra trạng thái kết nối với Laptop (Ground Station) và thực hiện các lệnh dừng khẩn cấp tại chỗ.

## 3. CẤU TRÚC CÁC NODE ROS 2 CHÍNH
Thay vì xử lý thuật toán nặng, Raspberry Pi tập trung vào việc quản lý các Node thu thập và chuyển tiếp:

| Tên Node | Vai trò chính | Dữ liệu Publish (Output) |
| :--- | :--- | :--- |
| **rplidar_node** | Đọc dữ liệu quét laser từ Lidar. | `/scan` (Dữ liệu khoảng cách 2D). |
| **usb_cam_node** | Thu thập luồng ảnh từ Webcam phía trước. | `/image_raw` (Ảnh gốc). |
| **image_transport** | Nén ảnh (JPEG/H.264) để truyền qua 4G. | `/image_raw/compressed`. |
| **uros_agent_node** | Cầu nối nhận dữ liệu từ ESP32-S3 và WROOM. | `/odom, /imu, /battery, /sensor_data`. |
| **safety_watchdog** | Giám sát nhịp tim (Heartbeat) từ Laptop. | `/cmd_vel` (Lệnh dừng nếu rớt mạng). |

## 4. QUẢN LÝ TRUYỀN THÔNG WAN (4G & TAILSCALE)
Do đặc thù truyền tải qua mạng di động có băng thông giới hạn, kiến trúc phần mềm áp dụng các cơ chế tối ưu:
* **Discovery Server:** Cấu hình ROS 2 sử dụng chế độ Discovery Server thay vì Multicast truyền thống để giảm thiểu lưu lượng mạng thừa trên kênh Tailscale.
* **Image Transport Protocol:** Sử dụng bộ nén ảnh động, tự động giảm chất lượng (bitrate) khi tín hiệu 4G yếu để duy trì độ trễ thấp cho việc xử lý YOLO trên Laptop.
* **DDS Tuning:** Tinh chỉnh các tham số QoS (Quality of Service):
    * *Sensors (Lidar/Camera):* Best Effort (Ưu tiên tốc độ, chấp nhận mất gói).
    * *Control (Lệnh di chuyển):* Reliable (Bắt buộc nhận đủ, đảm bảo an toàn).

## 5. LOGIC AN TOÀN & DỰ PHÒNG (FAIL-SAFE LOGIC)
Phần mềm trên Pi đóng vai trò là chốt chặn an toàn cuối cùng trước khi lệnh xuống động cơ:
* **Mất kết nối Laptop (WAN Fail):** Nếu Node `safety_watchdog` không nhận được dữ liệu từ Topic `/cmd_vel` (do Laptop gửi về) quá 500ms, Pi sẽ tự động gửi lệnh vận tốc bằng 0 xuống Slave.
* **Quản lý tài nguyên:** Sử dụng Cgroups để giới hạn tài nguyên CPU cho Node xử lý ảnh, ưu tiên băng thông tuyệt đối cho dữ liệu Lidar và micro-ROS Agent để tránh treo hệ thống dẫn đường.

## 6. TIÊU CHÍ NGHIỆM THU PHẦN MỀM MASTER
* **Độ trễ truyền tin:** Lệnh điều khiển từ Laptop qua 4G đến được Slave với độ trễ phản hồi tổng thể < 150ms (trong điều kiện sóng ổn định).
* **Tính ổn định:** Hệ thống Tailscale tự động kết nối lại (Auto-reconnect) khi xe di chuyển giữa các vùng phủ sóng 4G khác nhau mà không cần khởi động lại các Node ROS.
* **Hiệu suất:** Tải CPU của Raspberry Pi duy trì mức < 60% khi đang truyền tải đồng thời Lidar và Video nén, đảm bảo không gây nhiệt độ quá cao làm giảm xung nhịp xử lý.
