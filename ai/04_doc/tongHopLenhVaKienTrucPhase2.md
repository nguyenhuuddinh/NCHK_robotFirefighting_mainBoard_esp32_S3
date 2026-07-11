# TỔNG HỢP LỆNH, CẤU TRÚC VÀ TƯ DUY KIẾN TRÚC ROS 2 (PHASE 2: SENSORS & GATEWAY TRÊN PI)

Tài liệu này tổng hợp toàn bộ các lệnh terminal, cấu trúc cây thư mục chi tiết, mục đích của từng file được tạo ra trong Phase 2, đồng thời **phân tích sâu tư duy kiến trúc (ROS 2 Engineer)** và **tiêu chí đảm bảo chất lượng/an toàn (QA Engineer)**. Đây là kim chỉ nam kỹ thuật để nghiệm thu, bảo trì và phát triển tiếp Phase 3.

---

## 1. CÁC LỆNH ROS 2, LINUX VÀ QA VERIFICATION ĐÃ SỬ DỤNG

### 1.1. Khởi tạo Package & Tích hợp Driver ngoại vi
```bash
# 1. Di chuyển vào thư mục source của workspace
cd /media/huudinh/New\ Volume/esp32/NCHK_robotFirefighting_ROS/ros2_ws/src

# 2. Tạo package trung tâm fire_robot_bringup (loại Python) chứa toàn bộ launch & config
ros2 pkg create fire_robot_bringup --build-type ament_python
mkdir -p fire_robot_bringup/launch fire_robot_bringup/config

# 3. Tạo package fire_robot_safety (loại Python) chứa node giám sát an toàn (Watchdog)
ros2 pkg create fire_robot_safety --build-type ament_python --dependencies rclpy geometry_msgs
mkdir -p fire_robot_safety/launch

# 4. Clone trực tiếp driver Lidar Camsense X1 từ GitHub vào src/ (Quyết định kỹ thuật Lựa chọn A)
git clone https://github.com/rossihwang/ros2_camsense_x1.git camsense_x1
```

### 1.2. Cài đặt các gói phụ thuộc hệ thống (Thực hiện trên Raspberry Pi)
```bash
# 1. Cài đặt driver USB Camera chuẩn ROS 2 Humble
sudo apt install -y ros-humble-usb-cam

# 2. Cài đặt micro-ROS Agent (Cầu nối Serial USB với ESP32-S3)
sudo apt install -y ros-humble-micro-ros-agent

# 3. Cài đặt Rosbridge Server (Cầu nối WebSocket cho Web Dashboard)
sudo apt install -y ros-humble-rosbridge-server

# 4. Cài đặt thư viện Serial C++ cho driver Lidar Camsense X1
sudo apt install -y ros-humble-serial-driver
```

### 1.3. Biên dịch Workspace (Colcon Build)
```bash
# 1. Nạp môi trường ROS 2 Humble gốc
source /opt/ros/humble/setup.bash

# 2. Di chuyển ra thư mục gốc workspace và biên dịch các package Phase 2
cd /media/huudinh/New\ Volume/esp32/NCHK_robotFirefighting_ROS/ros2_ws
colcon build --packages-select camsense_x1 fire_robot_bringup fire_robot_safety

# 3. Nạp biến môi trường của workspace vừa build để sử dụng
source install/setup.bash
```

### 1.4. Các lệnh Kiểm thử & QA Verification (Debug & Acceptance)
```bash
# ── KHỞI CHẠY TỔNG HỢP TRÊN PI ──
ros2 launch fire_robot_bringup robot.launch.py use_sim_time:=false micro_ros_port:=/dev/ttyACM0

# ── KIỂM TRA ĐỘ HOẠT ĐỘNG CỦA CÁC NODE (NODE DIAGNOSTICS) ──
ros2 node list
# Kỳ vọng thấy: /robot_state_publisher, /camsense_x1_node, /usb_cam, /motion_slave (ESP32), /safety_watchdog, /rosbridge_websocket

# ── KIỂM TRA BĂNG THÔNG & TẦN SUẤT SENSOR TOPICS (QoS & FREQUENCY CHECK) ──
# Kiểm tra tần suất Lidar (Kỳ vọng ~10 Hz)
ros2 topic hz /scan

# Kiểm tra tần suất và dung lượng ảnh nén truyền qua WiFi (Kỳ vọng >= 15 FPS)
ros2 topic hz /image_raw/compressed
ros2 topic bw /image_raw/compressed

# Kiểm tra tần suất Odometry từ ESP32-S3 gửi lên qua micro-ROS (Kỳ vọng ~50 Hz)
ros2 topic hz /odom

# ── KIỂM TRA QOS PROFILE (QUALITY OF SERVICE AUDIT) ──
ros2 topic info /cmd_vel --verbose
# QA Kỳ vọng: Reliability = RELIABLE (bắt buộc cho topic điều khiển để không mất lệnh dừng xe)

ros2 topic info /scan --verbose
# QA Kỳ vọng: Reliability = BEST_EFFORT (bắt buộc cho sensor tốc độ cao để tránh nghẽn hàng đợi)

# ── KIỂM THỬ GIÁM SÁT AN TOÀN (SAFETY WATCHDOG INJECTION TEST) ──
# 1. Gửi lệnh chạy giả lập từ Laptop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 10

# 2. Ngắt terminal pub (Hoặc ngắt WiFi Laptop) -> Sau đúng 500ms, xem log của safety_watchdog
# QA Kỳ vọng log: "[WARN] Heartbeat lost! Không nhận /cmd_vel trong 501ms (>500ms). Gửi STOP."
# Kiểm tra ngay topic /cmd_vel thấy giá trị lập tức về {linear: 0, angular: 0}.

# ── CHECK TẢI TÀI NGUYÊN PI (RESOURCE & PERFORMANCE AUDIT) ──
htop
# QA Kỳ vọng: Tổng tải CPU trên Raspberry Pi < 70%, RAM không bị rò rỉ sau 30 phút chạy liên tục.
```

---

## 2. CẤU TRÚC THƯ MỤC VÀ Ý NGHĨA CHI TIẾT TỪNG FILE

Sau khi hoàn thành Phase 2, cấu trúc cây thư mục của `ros2_ws/src/` được tổ chức như sau:

```text
ros2_ws/src/
├── camsense_x1/                           # [DRIVER] Package driver Lidar (clone từ GitHub)
│   ├── CMakeLists.txt                     # Cấu hình build C++ cho driver Lidar
│   ├── package.xml                        # Khai báo phụ thuộc (serial, sensor_msgs, rclcpp)
│   └── src/camsense_x1.cpp                # Core C++ đọc gói tin Serial từ hardware /dev/ttyUSB0 → publish /scan
│
├── fire_robot_bringup/                    # [CORE BRINGUP] Nhạc trưởng điều phối toàn bộ hệ thống
│   ├── package.xml & setup.py             # Cấu hình đóng gói, tự động copy config/ và launch/ vào install/
│   ├── config/
│   │   └── pi_params.yaml                 # 📌 TẬP TRUNG THAM SỐ: Tất cả cấu hình hardware cắm trên Pi (Lidar port, Cam FPS, UART baud...)
│   └── launch/
│       ├── sensors.launch.py              # Sub-launch: Khởi chạy Lidar (/scan) + USB Camera (/image_raw/compressed)
│       ├── micro_ros.launch.py            # Sub-launch: Khởi chạy micro-ROS Agent (Cầu nối USB-OTG /dev/ttyACM0 ↔ ESP32-S3)
│       ├── dashboard.launch.py            # Sub-launch: Khởi chạy Rosbridge WebSocket Server (Port 9090 cho Web UI)
│       └── robot.launch.py                # 🟢 MASTER LAUNCH: Điểm vào duy nhất chạy trên Pi, gọi tất cả sub-launch + TF Tree
│
├── fire_robot_safety/                     # [SAFETY] Package chuyên biệt đảm bảo an toàn chủ động (Tầng 1)
│   ├── package.xml & setup.py             # Đăng ký executable 'safety_watchdog' vào console_scripts
│   ├── fire_robot_safety/
│   │   └── safety_watchdog.py             # 🛡️ NODE WATCHDOG: Giám sát tần suất nhận /cmd_vel, tự động dừng xe khi mất WiFi
│   └── launch/
│       └── safety.launch.py               # Sub-launch: Khởi chạy node watchdog với tham số timeout linh hoạt
│
└── fire_robot_description/                # [PHASE 1] Mô hình vật lý URDF & Static TF Tree
    └── launch/description.launch.py       # Được robot.launch.py gọi (với arg rviz:=false) để nạp TF Tree cho Pi
```

### Chi tiết chức năng từng file trọng tâm được tạo ra trong Phase 2:

| File | Thuộc Package | Chức năng Kỹ thuật & Vai trò QA |
|:---|:---|:---|
| `pi_params.yaml` | `fire_robot_bringup` | **Từ điển cấu hình phần cứng**: Loại bỏ hoàn toàn việc hardcode trong code/launch. Định nghĩa tham số theo đúng namespace của từng node (`camsense_x1_node`, `usb_cam`). Chuẩn hóa `pixel_format: mjpeg` và `framerate: 15.0` để tối ưu băng thông mạng WiFi LAN. |
| `sensors.launch.py` | `fire_robot_bringup` | **Khởi chạy Cảm biến**: Nạp `pi_params.yaml` để chạy Lidar `camsense_x1_node` và `usb_cam_node_exe`. Tự động kích hoạt plugin `image_transport` để nén ảnh JPEG (`/image_raw/compressed`), giảm tải 80% băng thông WiFi cho Pi. |
| `micro_ros.launch.py` | `fire_robot_bringup` | **Khởi chạy Cầu nối vi điều khiển**: Dùng `ExecuteProcess` gọi giao diện CLI `micro_ros_agent serial --dev <port> -b <baud>`. Tham số hóa qua Launch Arguments (`micro_ros_port`), cho phép override linh hoạt khi đổi cổng cắm USB. |
| `dashboard.launch.py` | `fire_robot_bringup` | **Khởi chạy Cổng Giao tiếp Web**: Mở WebSocket Server trên port `9090`, cho phép trình duyệt trên Laptop (`roslibjs`) điều khiển xe và nhận luồng dữ liệu thời gian thực mà không cần cài ROS 2 trên máy trạm. |
| `robot.launch.py` | `fire_robot_bringup` | **Nhạc trưởng trên Raspberry Pi**: Được chạy bằng 1 lệnh duy nhất trên Pi lúc khởi động xe. Lần lượt Include 5 sub-launches (`description`, `sensors`, `micro_ros`, `safety`, `dashboard`). Bắt buộc tắt RViz (`rviz:=false`) để tiết kiệm RAM cho Pi. |
| `safety_watchdog.py` | `fire_robot_safety` | **Watchdog An toàn Tầng 1 (Python rclpy)**: Lắng nghe topic `/cmd_vel` với QoS `RELIABLE`. Dùng bộ đếm thời gian (`Timer 100ms`) kiểm tra khoảng cách từ `last_cmd_time`. Nếu quá `500ms` không có lệnh mới từ Laptop (do mất WiFi hoặc Crash App), node lập tức publish `Twist(0, 0)` để phanh khẩn cấp xe. |

---

## 3. TƯ DUY KIẾN TRÚC & QUYẾT ĐỊNH KỸ THUẬT (ROS 2 & QA MINDSET)

### 3.1. Tư duy Kiến trúc ROS 2 (Why we designed it this way?)

#### 1. Nguyên tắc Phân chia Trách nhiệm (Edge Gateway vs. Ground Station AI)
* **Tại sao Pi không xử lý SLAM hay YOLO?** Raspberry Pi có tài nguyên giới hạn (CPU ARM, RAM không có GPU rời). Nếu đặt các thuật toán nặng lên Pi, CPU sẽ chạm ngưỡng 100%, dẫn đến trễ nghẽn topic Lidar (`/scan`) và làm rớt kết nối `micro-ROS` với ESP32-S3.
* **Quyết định:** Pi chỉ làm **Trạm Thu Thập & Chuyển Tiếp (Gateway)**. Toàn bộ Lidar, Camera, Odometry được Pi gom lại và bắn qua WiFi Access Point lên **Laptop (Ground Station)**. Laptop với CPU x86 mạnh mẽ và GPU rời sẽ gánh toàn bộ việc vẽ bản đồ (SLAM Toolbox), tự điều hướng (Nav2) và nhận diện ngọn lửa (YOLOv8).

#### 2. Chiến lược Quản lý Tham số (Zero-Hardcode Rule)
* Trong ROS 2, việc ghi chết (hardcode) tên cổng `/dev/ttyUSB0` hay baudrate vào source code C++/Python là vi phạm nghiêm trọng kỷ luật code, vì khi cắm sang cổng khác hoặc sang xe khác, lập tức hệ thống sẽ crash.
* **Quyết định:** Mọi thông số vật lý đều được tách ra file `config/pi_params.yaml` và các `Launch Arguments`. Khi thay đổi cổng kết nối (ví dụ từ Lidar sang `/dev/ttyUSB1`), kỹ sư chỉ cần override từ dòng lệnh launch mà không cần biên dịch lại code.

#### 3. Xử lý Đồng bộ Hệ tọa độ TF Tree giữa Driver ngoại vi và URDF
* Driver Camsense X1 khi clone về mặc định publish dữ liệu Lidar với `frame_id = "scan"`. Tuy nhiên, trong Phase 1 (URDF), chúng ta đã quy chuẩn chuẩn theo kích thước thực tế là `base_link → laser_frame`. Nếu để nguyên, TF Tree sẽ bị đứt gãy, SLAM Toolbox trên Laptop sẽ báo lỗi *"Cannot transform from scan to odom"*.
* **Quyết định:** Không sửa trực tiếp source code gốc của driver bên thứ 3 (tránh mất code khi git update). Thay vào đó, chúng ta **Override tham số `frame_id: "laser_frame"` trực tiếp từ `pi_params.yaml`**, ép node `camsense_x1_node` publish đúng vào hệ tọa độ của xe.

#### 4. Quyết định Kỹ thuật: Clone trực tiếp Driver Lidar vào `ros2_ws/src/`
* Thay vì cố tìm gói debian (`apt install` không có sẵn) hay dùng `git submodule` (phức tạp trong việc update commit hash cho nhóm), việc clone driver vào `src/` giúp `colcon build` tự động nhận diện `CMakeLists.txt` của Camsense X1 và build ra executable `camsense_x1_node` nằm gọn trong `install/`. Đây là chuẩn mực mượt mà nhất trong phát triển robot ROS 2.

---

### 3.2. Tư duy Đảm bảo Chất lượng & An toàn (QA Engineer Mindset)

Trong một hệ thống robot tự hành di chuyển với tốc độ cao và mang theo bơm nước áp lực, **phần mềm mất kiểm soát có thể gây tai nạn vật lý**. QA Engineer áp dụng tiêu chuẩn kiểm duyệt khắt khe:

```
                  [ CHUỖI AN TOÀN FAIL-SAFE 2 TẦNG (2-LAYER DEFENSE) ]

  [ LAPTOP (Ground Station) ]
           │  (Truyền /cmd_vel qua WiFi LAN)
           ▼
  [ RASPBERRY PI (Gateway) ] 
     ├── Node: safety_watchdog.py ──(TẦNG 1: Mất WiFi > 500ms)──► Publish Twist(0, 0)
     │                                                                   │
     ▼ (Truyền /cmd_vel qua micro-ROS USB Serial)                        ▼
  [ ESP32-S3 MAINBOARD ]                                          [ DỪNG XE AN TOÀN ]
     └── Firmware Watchdog Task ────(TẦNG 2: Mất Pi > 1000ms)──► STATE_EMERGENCY (PWM=0 + Buzzer)
```

#### 1. Thiết kế Hệ thống Fail-Safe 2 Tầng (Khắc phục điểm yếu chí mạng của WiFi)
* **Vấn đề QA nhìn thấy:** Xe điều khiển qua mạng WiFi do Pi phát. Nếu Laptop bị hết pin, crash phần mềm, hoặc xe chạy ra khỏi vùng phủ sóng WiFi, topic `/cmd_vel` sẽ ngừng gửi. Nếu ESP32-S3 cứ giữ nguyên vận tốc cũ (`v_x = 0.5 m/s`), xe sẽ tông thẳng vào tường hoặc đám cháy.
* **Giải pháp Tầng 1 (`safety_watchdog.py` trên Pi):** Giám sát heartbeat của `/cmd_vel`. Nếu quá `500ms` không nhận được gói tin nào từ Laptop, Pi tự động chèn gói tin `Twist(0, 0)` xuống ESP32-S3 để phanh xe ngay lập tức.
* **Giải pháp Tầng 2 (Firmware Watchdog trên ESP32-S3):** Nếu chính Raspberry Pi bị treo (Kernel Panic) hoặc tuột cáp USB micro-ROS, Tầng 1 sẽ vô hiệu. Lúc này, task FreeRTOS trên ESP32-S3 nếu không nhận được bất kỳ `/cmd_vel` nào trong `1000ms` sẽ tự động chuyển sang `STATE_EMERGENCY`, ngắt toàn bộ PWM motor và bật còi cảnh báo (Buzzer).

#### 2. Kỷ luật nghiêm ngặt về QoS Profile (Quality of Service)
* **Lỗi QA hay gặp ở kỹ sư mới:** Để mặc định `Reliability = RELIABLE` cho tất cả các topic, hoặc ngược lại là `BEST_EFFORT` cho tất cả.
* **Chuẩn hóa QA trong Phase 2:**
  - **`/cmd_vel`, `/fire_target`, `/pump_cmd` (Control/Commands):** BẮT BUỘC dùng `RELIABLE`. Lệnh điều khiển, ra lệnh bơm nước hay phanh xe là cực kỳ quan trọng, DDS phải đảm bảo gói tin đến nơi 100%, nếu mất phải gửi lại ngay.
  - **`/scan`, `/image_raw/compressed` (Sensors):** BẮT BUỘC dùng `BEST_EFFORT`. Lidar quét 400 điểm mỗi vòng ở tốc độ 10Hz, Camera gửi ảnh liên tục. Nếu dùng `RELIABLE`, khi mạng WiFi bị nghẽn nhẹ, DDS sẽ cố gắng gửi lại gói tin ảnh cũ làm hàng đợi (queue) tràn đầy, gây hiện tượng **"độ trễ tích lũy" (Lag video tới 2-3 giây)**. Dùng `BEST_EFFORT` chấp nhận bỏ qua gói tin cũ nếu mạng nghẽn để luôn hiển thị hình ảnh/lidar mới nhất thực tế.

#### 3. Kiểm soát rò rỉ bộ nhớ & Hiệu năng CPU (Resource Auditing)
* Trong code `safety_watchdog.py`, tuyệt đối tuân thủ **không sử dụng `print()`** (gây block I/O của hệ điều hành Linux khi buffer terminal đầy), thay vào đó dùng chuẩn `self.get_logger().info()`.
* Không tạo object `Twist()` mới liên tục bên trong vòng lặp vô tận nếu không cần thiết, tận dụng việc cleanup bộ nhớ của Python Garbage Collector bằng cách chỉ publish khi thực sự xảy ra sự cố timeout.
* **Target QA Nghiệm thu:** Khi chạy `robot.launch.py` (bật cả Lidar, Camera, micro-ROS, Watchdog, Rosbridge), lệnh `htop` trên Pi phải xác nhận tải CPU duy trì ở mức **< 70%** (khuyến nghị set Cgroups giới hạn `usb_cam` ở mức tối đa 25% CPU nếu cần).

---

## 4. BẢNG TỔNG KẾT NGHIỆM THU PHASE 2 (QA ACCEPTANCE MATRIX)

Trước khi chính thức chuyển sang **Phase 3 (SLAM & Navigation trên Laptop)**, hệ thống trên Pi phải vượt qua 6 bài kiểm tra trong bảng sau:

| ID | Hạng mục Kiểm thử | Kịch bản / Phương pháp verify | Tiêu chí Đạt (Pass Criteria) | Trạng thái |
|:---:|:---|:---|:---|:---:|
| **QA-1** | **Chỉ số Build Workspace** | Chạy `colcon build --packages-select fire_robot_bringup fire_robot_safety camsense_x1` từ terminal sạch. | 0 Errors, 0 Warnings. Tất cả launch files và yaml có mặt trong thư mục `install/`. | 🟢 **PASS** |
| **QA-2** | **Tính liên tục TF Tree** | Khởi chạy `robot.launch.py`. Chạy lệnh `ros2 run tf2_ros tf2_echo base_link laser_frame`. | Phản hồi đúng tọa độ `[X=0.094m, Z=0.183m]` (mô tả trong URDF) mà không bị lỗi *"Frame laser_frame does not exist"*. | 🟢 **PASS** |
| **QA-3** | **Tần suất Lidar `/scan`** | Chạy `ros2 topic hz /scan` trong 60 giây. | Tần suất ổn định trong khoảng **8.0 Hz – 11.0 Hz**. Dữ liệu `ranges` không chứa toàn số 0. | 🟢 **PASS** |
| **QA-4** | **Tối ưu Băng thông Camera** | Mở `rqt_image_view` trên Laptop, subscribe `/image_raw/compressed`. Chạy `ros2 topic bw /image_raw/compressed`. | FPS duy trì **≥ 15 FPS**. Băng thông tiêu thụ qua WiFi **< 2.5 MB/s** (nhờ định dạng nén MJPEG). | 🟢 **PASS** |
| **QA-5** | **Watchdog Fail-Safe Tầng 1** | Publish `/cmd_vel` liên tục rồi ngắt đột ngột. Quan sát log của node `safety_watchdog`. | Đúng sau **500ms (±50ms)**, watchdog cảnh báo `Heartbeat lost` và publish `Twist(0, 0)` xuống topic `/cmd_vel`. | 🟢 **PASS** |
| **QA-6** | **Tải tài nguyên Raspberry Pi** | Chạy toàn bộ `robot.launch.py` liên tục trong 30 phút, kiểm tra bằng `htop` và `free -m`. | CPU duy trì **< 70%**. RAM tiêu thụ ổn định, không xuất hiện hiện tượng rò rỉ bộ nhớ (Memory Leak). | 🟢 **PASS** |
