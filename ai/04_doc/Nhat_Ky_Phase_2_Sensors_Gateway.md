# NHẬT KÝ PHÁT TRIỂN ROS 2 - PHASE 2 (SENSORS, SAFETY GATEWAY & MICRO-ROS DEBUG ĐẾN BƯỚC 4.2)

> **Mục đích tài liệu:** Ghi chép lại toàn bộ hành trình thực hiện Phase 2 dưới góc nhìn của **ROS 2 Engineer** và **QA Engineer (`qa_engineer.md`)**. Tài liệu tổng hợp cách tư duy kiến trúc, lý do đằng sau các quyết định kỹ thuật, những câu prompt/hỏi-đáp thực tế trong quá trình pair-programming, cùng toàn bộ các lỗi (bug) môi trường và phần cứng đã gặp phải từ Laptop đến Raspberry Pi và ESP32-S3 (đến nghiệm thu Bước 4.2).

---

## 1. TƯ DUY KIẾN TRÚC & QUYẾT ĐỊNH KỸ THUẬT (ROS 2 ENGINEER + QA ENGINEER)

### 1.1. Phân tách Gateway (Raspberry Pi) vs Ground Station (Laptop)
* **Quyết định:** Raspberry Pi chỉ đóng vai trò là **Edge Gateway** (Trạm trung chuyển dữ liệu ở biên). Toàn bộ thuật toán nặng như định vị SLAM, dẫn đường Nav2, nhận diện lửa bằng YOLOv8 đều phải chạy trên **Ground Station (Laptop)**.
* **Tư duy & Lý do (QA Audit):**
  - **Tránh nghẽn cổ chai CPU:** Raspberry Pi (kể cả Pi 4 hay Pi 5) khi gánh đồng thời Lidar 10Hz, Camera 15fps, cộng thêm thuật toán tối ưu hóa đồ thị (Graph-based SLAM) hoặc mạng Neural (YOLO) sẽ khiến tải CPU vượt $> 90\%$.
  - **Bảo vệ kết nối vi điều khiển:** Khi CPU Pi bị nghẽn hoặc trễ ngắt (interrupt latency), các gói tin DDS/XRCE từ ESP32-S3 gửi lên qua cổng Serial sẽ bị rớt (dropped frames), dẫn đến mất đồng bộ nhịp tim (heartbeat) và làm kích hoạt sai chế độ dừng khẩn cấp (Emergency Stop).

### 1.2. Chiến lược Zero-Hardcode & Quy hoạch tham số (`pi_params.yaml`)
* **Quyết định:** Không viết chết (hardcode) bất kỳ tham số nào như tên cổng (`/dev/video0`, `/dev/ttyACM0`), tốc độ baud (`115200`), hay hệ tọa độ (`frame_id`) bên trong mã nguồn C++/Python.
* **Tư duy:** Mọi thông số phần cứng được gom về file `config/pi_params.yaml` của package `fire_robot_bringup` và cho phép ghi đè linh hoạt qua **Launch Arguments** (`micro_ros_port`, `micro_ros_baudrate`). Khi cắm sang cổng USB khác hoặc đổi thiết bị, chỉ cần truyền tham số lúc gọi lệnh launch mà không cần dịch lại code (`colcon build`).

### 1.3. Giải pháp tích hợp Driver Lidar Camsense X1 (Lựa chọn A)
* **Quyết định:** Clone trực tiếp mã nguồn driver `rossihwang/ros2_camsense_x1` vào thư mục `ros2_ws/src/` để `colcon` tự biên dịch cùng toàn bộ workspace.
* **Quyết định Override TF Tree:** Thay vì sửa file C++ của driver để đổi tên frame từ `"laser"` thành `"laser_frame"`, ta sử dụng tính năng **Parameter Override** ngay trong `pi_params.yaml`:
  ```yaml
  camsense_x1_node:
    ros__parameters:
      frame_id: "laser_frame" # Đồng bộ hoàn hảo với URDF Phase 1
  ```
* **Lý do:** Giữ nguyên tính toàn vẹn của mã nguồn bên thứ 3 (Dễ dàng `git pull` cập nhật driver khi tác giả có bản patch mới) mà vẫn đảm bảo cây tọa độ TF (`base_link -> laser_frame`) không bị đứt gãy.

### 1.4. Hệ thống An toàn 2 Tầng (2-Layer Fail-Safe System)
Toàn bộ hệ thống được bảo vệ nhiều tầng nhằm chống hiện tượng robot "chạy trốn" (runaway robot) khi mất tín hiệu điều khiển:
* **Tầng 1 (Trên Raspberry Pi - `safety_watchdog.py`):** Node Watchdog lắng nghe topic `/cmd_vel`. Nếu sau $500\text{ms}$ không có bản tin điều khiển mới từ Laptop (do rớt WiFi hoặc Lag), Watchdog tự động phát bản tin `Twist(0, 0)` với QoS `RELIABLE` xuống topic `cmd_vel` để dừng xe.
* **Tầng 2 (Trên ESP32-S3 Firmware):** Nếu Raspberry Pi sập nguồn, treo hệ điều hành hoặc tuột cáp USB, ESP32-S3 không nhận được bản tin qua micro-ROS trong $1000\text{ms}$ sẽ tự động chuyển sang trạng thái `STATE_EMERGENCY`, ngắt toàn bộ xung PWM động cơ về 0 và bật còi báo động (Buzzer).

---

## 2. HÀNH TRÌNH HỎI - ĐÁP & TƯ DUY GIẢI QUYẾT VẤN ĐỀ (PROMPTS & ANSWERS)

Dưới đây là ghi chép chân thực về các câu hỏi (prompt) của User trong suốt quá trình triển khai Phase 2 và cách tư duy trả lời, hướng dẫn kỹ thuật của AI.

### ❓ Hỏi đáp 1: Cổng USB-OTG vs Cổng CP2102 trên ESP32-S3
* **Prompt của User:** *"ủa mà trên esp32 s3 có 2 cổng type c á 1 để nạp code với 1 cái type usb gì á thì nên lắp cổng nào vậy bạn... cổng usb otg này tôi tưởng là để cắm vào pi chứ còn cổng có cp210 thì để nạp code chứ, tại cái usb otg đâu có nạp code đc đâu bạn"*
* **Tư duy & Giải đáp:**
  - **Phân tích phần cứng:** Board ESP32-S3 (như dòng NodeMCU hay DevKitC) có 2 cổng Type-C: 1 cổng nối qua chip chuyển đổi UART (CP2102/CH340) và 1 cổng nối trực tiếp vào chân USB Native của chip S3 (USB-OTG / USB Serial/JTAG).
  - **Sự nhầm lẫn của User:** User nghĩ cổng OTG không nạp được code. Thực tế, cổng OTG của S3 hoàn toàn vừa nạp được firmware (qua giao tiếp D+/D- Native), vừa debug và vừa truyền dữ liệu tốc độ cực cao.
  - **Quyết định cho Robot:** Khi gắn ESP32-S3 lên Raspberry Pi làm vi điều khiển điều khiển xe, **khuyên dùng cổng USB-OTG (`/dev/ttyACM0`)** để chạy `micro-ROS Agent`. Cổng Native USB có độ trễ cực thấp (bỏ qua tầng đệm của chip CP2102) và giảm rủi ro rớt gói tin DDS. Cổng CP2102 (`/dev/ttyUSB0`) có thể để trống hoặc dùng cắm vào Laptop khi cần monitor Serial log song song lúc debug.

### ❓ Hỏi đáp 2: Clone workspace trên Laptop hay trên Pi?
* **Prompt của User:** *"ủa chứ k phải clone về làm full hết space trên laptop luôn rồi hồi sau, ssh pi clone nguyên cái space của tôi về rồi chạy launch thôi hã bạn"*
* **Tư duy & Giải đáp:**
  - **Xác nhận:** Đúng chính xác 100% với tư duy chuẩn kỹ sư phần mềm (Software Engineering Best Practice)!
  - **Quy trình chuẩn:** Không bao giờ code trực tiếp trên Raspberry Pi vì trình soạn thảo trên Pi chậm, khó quản lý version control và dễ mất dữ liệu nếu hỏng thẻ nhớ SD.
  - **Quy trình đã áp dụng:** Toàn bộ package (`fire_robot_bringup`, `fire_robot_safety`, config YAML, launch files) được viết, cấu trúc và kiểm thử cú pháp trên **Laptop (`huudinh@...`)**. Sau khi hoàn chỉnh, ta khởi tạo Git, đẩy (`push`) lên GitHub. Trên **Raspberry Pi (`pi@ubuntu`)**, ta chỉ cần `git clone`, gõ `colcon build` và chạy `ros2 launch` là toàn bộ hệ thống sống dậy.

### ❓ Hỏi đáp 3: Cách test kết nối Node micro-ROS từ ESP32-S3 lên Pi ở Bước 4.2
* **Prompt của User:** *"ở @[ai/03_templates/esp32_s3_step_by_step.md] tôi đã hoàn thành tới 4.2 rồi chừ đang muốn lệnh để test trên pi để nhân node từ esp32 á thì làm như nào bạn"*
* **Tư duy & Giải đáp:**
  - Ở Bước 4.2, firmware ESP32-S3 mới chỉ khởi tạo Transport (Serial USB), tạo Node (`motion_slave`) và Executor, chưa có Publisher hay Subscriber nào.
  - Để test nghiệm thu bước này, cần hướng dẫn User chạy `micro_ros_agent` trên Pi, kiểm tra log bắt tay (Ping/Pong Handshake) xem có báo `session established` hay không, và mở tab thứ 2 gõ `ros2 node list` để kiểm chứng sự xuất hiện của `/motion_slave`.

---

## 3. CÁC BUG MÔI TRƯỜNG & HỆ THỐNG ĐÃ GẶP VÀ CÁCH KHẮC PHỤC TRIỆT ĐỂ

Trong quá trình đưa code từ Laptop lên GitHub và xuống Raspberry Pi để chạy nghiệm thu Bước 4.2, hệ thống đã gặp 3 lỗi lớn. Dưới đây là phân tích nguyên nhân gốc rễ (Root Cause Analysis) và cách xử lý.

### 🐞 Bug 1: Lỗi `Package 'fire_robot_bringup' not found` trên Raspberry Pi
* **Tình trạng:** Khi gõ lệnh launch trên Terminal của Pi (`pi@ubuntu:~$`):
  ```bash
  ros2 launch fire_robot_bringup micro_ros.launch.py micro_ros_port:=/dev/ttyACM0
  ```
  Hệ thống báo lỗi đỏ:
  `Package 'fire_robot_bringup' not found: "package 'fire_robot_bringup' not found, searching: ['/opt/ros/humble']"`
* **Nguyên nhân gốc rễ:**
  1. Package `fire_robot_bringup` vừa được tạo ra và code ở Phase 2 trên **Laptop**, Pi chưa hề có thư mục dự án này.
  2. Ngay cả khi đã copy code sang, User trên Pi mới chỉ nạp môi trường ROS 2 gốc (`source /opt/ros/humble/setup.bash`) mà chưa nạp biến môi trường của workspace (`source ~/NCHK_robotFirefighting_ROS/ros2_ws/install/setup.bash`).
* **Cách khắc phục:**
  - Thiết lập Git repo cho dự án trên Laptop.
  - Tạo file `.gitignore` ở gốc để lọc bỏ toàn bộ thư mục `build/`, `install/`, `log/` và thư mục `ai/` (chứa AI skills local không cần đẩy lên Pi):
    ```text
    ai/
    ros2_ws/build/
    ros2_ws/install/
    ros2_ws/log/
    ```
  - Đẩy code lên GitHub, sang Pi gõ `git clone`, `colcon build` và `source install/setup.bash`.

---

### 🐞 Bug 2: Lỗi `ERROR: Repository not found` khi `git push` bằng SSH từ Laptop
* **Tình trạng:** Khi User cấu hình Git remote bằng SSH và đẩy code lên:
  ```bash
  git remote set-url origin git@github.com:huudinh/NCHK_robotFirefighting_ROS.git
  git push -u origin main
  ```
  Terminal báo lỗi:
  `ERROR: Repository not found. fatal: Could not read from remote repository.`
* **Nguyên nhân gốc rễ:**
  - AI đã chạy lệnh kiểm tra `ssh -T git@github.com` trên Laptop và phát hiện SSH Key trên Laptop đang được gắn với tài khoản GitHub mang tên **`nguyenhuuddinh`** (chứ không phải `huudinh`).
  - Hơn nữa, qua kiểm tra bằng GitHub API (`curl -s https://api.github.com/users/nguyenhuuddinh/repos`), tài khoản `nguyenhuuddinh` lúc đó chỉ có 6 repository cũ, **chưa hề tạo kho `NCHK_robotFirefighting_ROS` trên web GitHub.com**.
* **Cách khắc phục:**
  1. User truy cập `https://github.com/new` trên tài khoản `nguyenhuuddinh`, tạo kho trống tên chính xác là `NCHK_robotFirefighting_ROS` (không chọn tạo README hay .gitignore).
  2. Sửa lại URL remote cho đúng tên tài khoản và đẩy code:
     ```bash
     git remote set-url origin git@github.com:nguyenhuuddinh/NCHK_robotFirefighting_ROS.git
     git push -u origin main
     ```
  3. **Kết quả:** Push thành công mỹ mãn với `Writing objects: 100% (42/42), 13.82 KiB`.

---

### 🐞 Bug 3: Lỗi `Package 'micro_ros_agent' not found` và `process has died` khi chạy Launch file trên Pi
* **Tình trạng:** Sau khi Pi clone code về, `colcon build` thành công 3 package và gõ lệnh launch:
  ```bash
  ros2 launch fire_robot_bringup micro_ros.launch.py micro_ros_port:=/dev/ttyACM0
  ```
  Terminal Pi báo lỗi tiến trình bị chết ngay tức khắc:
  ```text
  [ros2-1] Package 'micro_ros_agent' not found
  [ERROR] [ros2-1]: process has died [pid 2084, exit code 1, cmd 'ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200'].
  ```
  User thử cài bằng Snap (`sudo snap install micro-ros-agent`) rồi chạy lại launch file vẫn lỗi, gõ trực tiếp `micro-ros-agent` thì báo `command not found`.
* **Nguyên nhân gốc rễ:**
  1. Gói `micro_ros_agent` không có sẵn trong lệnh `apt install ros-humble-...` của Ubuntu mặc định.
  2. Khi cài qua Snap, Snap đặt file thực thi vào `/snap/bin/micro-ros-agent` độc lập, **không đăng ký vào chỉ mục package (`ament_index`) của ROS 2**. Do đó, lệnh `ros2 run micro_ros_agent ...` nằm bên trong `micro_ros.launch.py` sẽ không thể tìm thấy package.
* **Cách khắc phục triệt để (Chuẩn kỹ sư ROS 2):**
  Xây dựng (build) `micro_ros_agent` từ mã nguồn chính chủ ngay bên trong một workspace riêng trên Pi (`~/microros_ws`):
  ```bash
  # 1. Nạp môi trường Humble
  source /opt/ros/humble/setup.bash

  # 2. Tạo workspace và tải công cụ micro_ros_setup
  mkdir -p ~/microros_ws/src && cd ~/microros_ws
  git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

  # 3. Cài tool và build setup
  sudo apt update && sudo apt install -y python3-vcstool build-essential
  colcon build && source install/local_setup.bash

  # 4. Tải source code agent và biên dịch vào ROS 2 Humble (mất ~4 phút trên Pi)
  ros2 run micro_ros_setup create_agent_ws.sh
  ros2 run micro_ros_setup build_agent.sh
  ```
* **Kết quả build trên Pi:**
  `Summary: 2 packages finished [4min 42s]` (`micro_ros_msgs` và `micro_ros_agent` hoàn tất 100%).
  Nạp môi trường: `source ~/microros_ws/install/local_setup.bash`. Từ thời điểm này, toàn bộ Launch file của Phase 2 chạy mượt mà vĩnh viễn!

---

## 4. NGHIỆM THU BƯỚC 4.2 CỦA ESP32-S3 (QA ACCEPTANCE MATRIX)

Sau khi xử lý hoàn tất các lỗi môi trường, tiến hành kiểm thử nghiệm thu **Bước 4.2 (Khởi tạo kết nối & Đăng ký Node `motion_slave` trên ESP32-S3)** theo checklist của QA Engineer.

### 4.1. Kịch bản chạy thực tế trên Raspberry Pi
**Tab 1 (Khởi chạy Agent qua Launch File Phase 2):**
```bash
source ~/microros_ws/install/local_setup.bash
cd ~/NCHK_robotFirefighting_ROS/ros2_ws && source install/setup.bash
ros2 launch fire_robot_bringup micro_ros.launch.py micro_ros_port:=/dev/ttyACM0 micro_ros_baudrate:=115200
```
*Nhật ký Terminal hiển thị khi bấm nút RESET (RST) trên ESP32-S3:*
```text
[ros2-1] [1783757967.038597] info | TermiosAgentLinux.cpp | init | running... | fd: 3
[ros2-1] [1783757967.441953] info | Root.cpp | create_client | create | client_key: 0x40E80F20, session_id: 0x81
[ros2-1] [1783757967.442111] info | SessionManager.hpp | establish_session | session established | client_key: 0x40E80F20, address: 0
[ros2-1] [1783757967.503443] info | ProxyClient.cpp | create_participant | participant created | client_key: 0x40E80F20, participant_id: 0x000(1)
```
👉 **Đánh giá QA:** Thông báo `session established` và `participant created` xác nhận bắt tay DDS XRCE-DDS thành công tuyệt đối qua cổng USB Native `/dev/ttyACM0`.

**Tab 2 (Kiểm tra ROS 2 Graph):**
```bash
source /opt/ros/humble/setup.bash
ros2 node list
```
*Kết quả xuất ra:*
```text
/motion_slave
```
*Kiểm tra chi tiết node:*
```bash
ros2 node info /motion_slave
```
*Kết quả xuất ra:*
```text
/motion_slave
  Subscribers:
  Publishers:
  Service Servers:
  Service Clients:
  Action Servers:
  Action Clients:
```
👉 **Đánh giá QA:** Node `/motion_slave` đã đăng ký thành công vào mạng lưới ROS 2 Humble của Raspberry Pi. Việc danh sách Publishers và Subscribers đang trống khớp $100\%$ với thiết kế của Bước 4.2 (chỉ mới init Node và Executor, chưa add topic).

---

## 5. TỔNG KẾT TRẠNG THÁI & CHUẨN BỊ CHO BƯỚC TIẾP THEO

| Hạng mục Kiểm định | Trạng thái | Ghi chú / Minh chứng |
| :--- | :---: | :--- |
| **Quy hoạch tham số (`pi_params.yaml`)** | 🟢 **PASSED** | Tham số hóa toàn bộ `frame_id`, `port`, `baudrate`. |
| **Fail-safe Safety Watchdog (`safety.launch.py`)** | 🟢 **PASSED** | Tự động `Twist(0,0)` sau 500ms mất tín hiệu. |
| **Cấu trúc Git & Clean Build (.gitignore)** | 🟢 **PASSED** | Repo sạch sẽ 13.82 KiB, bỏ qua `build/`, `install/`, `ai/`. |
| **Biên dịch `micro_ros_agent` trên Pi** | 🟢 **PASSED** | Build từ source vào `~/microros_ws`, tích hợp chuẩn `ros2 run`. |
| **Nghiệm thu Bước 4.2 ESP32-S3 (`/motion_slave`)** | 🟢 **PASSED** | `session established` & xuất hiện trên `ros2 node list`. |

### 🎯 Lộ trình tiếp theo (Tự debug và cập nhật sau):
* **Bước 4.3:** Thêm Publisher `/odom` ($50\text{ Hz}$) vào ESP32-S3 $\rightarrow$ Kiểm chứng bằng `ros2 topic hz /odom` và `ros2 topic echo /odom`.
* **Bước 4.4:** Thêm Publisher `/imu/data` $\rightarrow$ Kiểm chứng gia tốc và góc nghiêng khi lắc xe.
* **Bước 4.5:** Thêm Publisher `/env_status` $\rightarrow$ Kiểm chứng JSON cảm biến môi trường (Lửa, Gas, Nhiệt độ, Pin) gửi từ slave WROOM qua UART.
* **Bước 4.6:** Thêm TF Broadcaster (`odom -> base_link`) $\rightarrow$ Kiểm chứng bằng `ros2 run tf2_ros tf2_echo odom base_link`.
* **Bước 4.7:** Thêm Subscriber `/cmd_vel` $\rightarrow$ Thử nghiệm phát lệnh chạy xe từ Pi qua lệnh `ros2 topic pub`.
* **Bước 4.8:** Thêm Subscriber `/fire_target` & `/pump_cmd` $\rightarrow$ Thử nghiệm điều khiển Servo và rơ-le bơm nước chữa cháy.
