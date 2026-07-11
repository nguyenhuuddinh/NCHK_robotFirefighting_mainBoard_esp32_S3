# 🔧 QUY TRÌNH XÂY DỰNG ROS 2 WORKSPACE — TỪNG BƯỚC NHỎ + DEBUG

> [!IMPORTANT]
> **Nguyên tắc vàng:** Mỗi bước phải **build thành công + chạy đúng** trước khi qua bước tiếp. Nếu sai → fix ngay tại bước đó, KHÔNG tiến tiếp.

---

## PHASE 1: KHỞI TẠO WORKSPACE & ROBOT DESCRIPTION

### Bước 1.1: Tạo ROS 2 Workspace
**Làm gì:** Tạo thư mục `ros2_ws/src/` và khởi tạo các package.
- `mkdir -p ros2_ws/src`
- Tạo `.gitignore` cho ros2_ws (ignore `build/`, `install/`, `log/`)
- Tạo `README.md` hướng dẫn build

**🧪 Debug 1.1:**
```
✅ Thư mục ros2_ws/src/ tồn tại
✅ .gitignore chứa build/, install/, log/
```

---

### Bước 1.2: Tạo package fire_robot_description
**Làm gì:** `ros2 pkg create fire_robot_description --build-type ament_python`
- Tạo thư mục `urdf/`, `meshes/`, `launch/`, `config/`
- Viết URDF cơ bản: `base_link` (hình hộp) + `laser_frame` + `camera_frame` + `imu_frame`
- Static transforms dựa trên kích thước thực của xe

**🧪 Debug 1.2:**
```
✅ colcon build --packages-select fire_robot_description → pass
✅ ros2 launch fire_robot_description description.launch.py
✅ RViz hiển thị xe hình hộp + 3 frame (laser, camera, imu)
✅ ros2 run tf2_tools view_frames → TF tree đúng: base_link → laser_frame, camera_frame, imu_frame
```

---

### Bước 1.3: Kiểm tra TF Tree hoàn chỉnh
**Làm gì:** Verify TF tree khớp với spec trong Tai_Lieu_So_4.

**🧪 Debug 1.3:**
```
✅ ros2 run tf2_ros tf2_echo base_link laser_frame → có translation + rotation
✅ ros2 run tf2_ros tf2_echo base_link camera_frame → có translation + rotation
✅ ros2 run tf2_ros tf2_echo base_link imu_frame → có translation + rotation
✅ Khoảng cách laser_frame so với base_link khớp vị trí lắp Lidar trên xe thật
```

> [!TIP]
> **Checkpoint Phase 1:** URDF + TF tree hoạt động. RViz hiển thị mô hình xe.

---

## PHASE 2: SENSORS TRÊN PI (Thu thập dữ liệu)

### Bước 2.1: Tạo package fire_robot_bringup
**Làm gì:** `ros2 pkg create fire_robot_bringup --build-type ament_python`
- Tạo thư mục `launch/`, `config/`
- Tạo file `pi_params.yaml` (camera resolution, lidar port, baudrate)

**🧪 Debug 2.1:**
```
✅ colcon build --packages-select fire_robot_bringup → pass
```

---

### Bước 2.2: Launch Lidar trên Pi
**Làm gì:** Viết `sensors.launch.py` — khởi chạy `rplidar_ros`.
- Cài rplidar_ros: `sudo apt install ros-humble-rplidar-ros`
- Cấu hình: serial port (`/dev/ttyUSB0`), baudrate, frame_id (`laser_frame`)
- QoS: Best Effort cho `/scan`

**🧪 Debug 2.2 (chạy trên Pi):**
```
✅ ros2 launch fire_robot_bringup sensors.launch.py
✅ ros2 topic list → thấy /scan
✅ ros2 topic echo /scan → có dữ liệu LaserScan (ranges không toàn 0)
✅ ros2 topic hz /scan → ~10 Hz
✅ Trên Laptop (cùng WiFi): ros2 topic echo /scan → nhận được data
```

---

### Bước 2.3: Launch Camera trên Pi
**Làm gì:** Thêm `usb_cam` vào `sensors.launch.py`.
- Cài usb_cam: `sudo apt install ros-humble-usb-cam`
- Cấu hình: device (`/dev/video0`), resolution (640x480), FPS, pixel format
- Bật `image_transport` compressed

**🧪 Debug 2.3 (chạy trên Pi):**
```
✅ ros2 topic list → thấy /image_raw, /image_raw/compressed
✅ ros2 topic hz /image_raw/compressed → ≥ 10 FPS
✅ Trên Laptop: ros2 run rqt_image_view rqt_image_view → chọn /image_raw/compressed → thấy hình
✅ Bandwidth test: không làm Lidar bị lag (ros2 topic hz /scan vẫn ~10Hz)
```

---

### Bước 2.4: Launch micro-ROS Agent trên Pi
**Làm gì:** Viết `micro_ros.launch.py`.
- Cài micro-ROS Agent: `sudo apt install ros-humble-micro-ros-agent` (hoặc build từ source)
- Cấu hình: serial device (`/dev/ttyACM0` hoặc `/dev/ttyUSB1`), baudrate 115200

**🧪 Debug 2.4 (Pi + ESP32-S3 kết nối USB):**
```
✅ ros2 launch fire_robot_bringup micro_ros.launch.py
✅ Agent log: "Node 'motion_slave' connected"
✅ ros2 node list → /motion_slave
✅ ros2 topic list → thấy /odom, /imu/data, /env_status
✅ ros2 topic echo /odom → có dữ liệu Odometry
✅ Trên Laptop: ros2 topic pub /cmd_vel geometry_msgs/Twist "..." → xe chạy
```

---

### Bước 2.5: Safety Watchdog trên Pi
**Làm gì:** Tạo package `fire_robot_safety`, viết node `safety_watchdog.py`.
- Subscribe `/cmd_vel` → ghi nhận timestamp cuối cùng
- Timer 100ms: nếu `now - last_cmd > 500ms` → publish `/cmd_vel` = Twist(0,0)
- Log warning khi kích hoạt watchdog

**🧪 Debug 2.5:**
```
✅ colcon build → pass
✅ Chạy watchdog + gửi /cmd_vel từ Laptop → xe chạy bình thường
✅ Dừng gửi /cmd_vel → sau 500ms watchdog log "[WARN] Heartbeat lost, sending STOP"
✅ Xe dừng lại
✅ Gửi /cmd_vel lại → xe chạy tiếp, watchdog tắt cảnh báo
```

---

### Bước 2.6: Tổng hợp Launch file Pi
**Làm gì:** Viết `robot.launch.py` gọi tất cả: sensors + micro_ros + safety + rosbridge.

**🧪 Debug 2.6:**
```
✅ ros2 launch fire_robot_bringup robot.launch.py → khởi chạy 1 lệnh duy nhất
✅ ros2 node list → thấy: rplidar_node, usb_cam_node, micro_ros_agent, safety_watchdog, rosbridge_websocket
✅ Tất cả topic hoạt động bình thường
✅ CPU Pi < 70% (kiểm tra: htop)
```

> [!TIP]
> **Checkpoint Phase 2:** Pi thu thập đầy đủ Lidar + Camera + ESP32 data. Laptop nhận được tất cả qua WiFi.

---

## PHASE 3: SLAM & NAVIGATION TRÊN LAPTOP

### Bước 3.1: Tạo package fire_robot_navigation
**Làm gì:** `ros2 pkg create fire_robot_navigation --build-type ament_python`
- Tạo `config/slam_params.yaml`, `config/nav2_params.yaml`
- Cài: `sudo apt install ros-humble-slam-toolbox ros-humble-navigation2`

**🧪 Debug 3.1:**
```
✅ colcon build → pass
```

---

### Bước 3.2: SLAM — Vẽ bản đồ
**Làm gì:** Viết `slam.launch.py` khởi chạy `slam_toolbox` online async.
- Cấu hình: `/scan` topic, `base_frame: base_link`, `odom_frame: odom`
- Đảm bảo TF tree đầy đủ: odom→base_link (từ ESP32) + base_link→laser_frame (từ URDF)

**🧪 Debug 3.2 (Laptop kết nối WiFi Pi):**
```
✅ ros2 launch fire_robot_navigation slam.launch.py
✅ RViz: thấy bản đồ đang được vẽ dần từ dữ liệu Lidar
✅ Dùng teleop_twist_keyboard điều khiển xe đi quanh phòng → bản đồ mở rộng
✅ Bản đồ không bị "trượt" khi xe xoay tại chỗ (nhờ IMU fusion)
✅ Lưu bản đồ: ros2 run nav2_map_server map_saver_cli -f ~/map
```

---

### Bước 3.3: Nav2 — Tự điều hướng
**Làm gì:** Viết `nav2.launch.py` khởi chạy Nav2 stack.
- Cấu hình `nav2_params.yaml`: controller, planner, costmap, recovery behaviors
- Tối ưu cho skid-steer: `robot_radius` thay vì `footprint`, `min_vel_x: 0.0` (không đi lùi)

**🧪 Debug 3.3:**
```
✅ ros2 launch fire_robot_navigation nav2.launch.py map:=~/map.yaml
✅ RViz: click "2D Nav Goal" → xe tự đi đến điểm đích
✅ Xe tránh vật cản trên đường đi
✅ Xe dừng khi đến đích, tolerance < 0.1m
✅ Thử chặn đường → xe tìm đường khác hoặc recovery
```

---

### Bước 3.4: Localization (AMCL) — Dùng bản đồ đã lưu
**Làm gì:** Viết `localization.launch.py` — chạy AMCL thay SLAM.
- Dùng cho trường hợp đã có bản đồ, chỉ cần định vị xe.

**🧪 Debug 3.4:**
```
✅ ros2 launch fire_robot_navigation localization.launch.py map:=~/map.yaml
✅ RViz: "2D Pose Estimate" → đặt vị trí ban đầu → particles hội tụ
✅ Xe di chuyển → particles theo sát vị trí thực
```

> [!TIP]
> **Checkpoint Phase 3:** Xe tự vẽ bản đồ và tự điều hướng. SLAM + Nav2 hoạt động.

---

## PHASE 4: YOLO NHẬN DIỆN LỬA

### Bước 4.1: Tạo package fire_robot_perception
**Làm gì:** `ros2 pkg create fire_robot_perception --build-type ament_python`
- Tạo `config/yolo_params.yaml`: model_path, confidence_threshold, input_size
- Thêm dependency: `cv_bridge`, `sensor_msgs`

**🧪 Debug 4.1:**
```
✅ colcon build → pass
```

---

### Bước 4.2: YOLO Fire Detector Node
**Làm gì:** Viết `yolo_fire_detector.py`.
- Subscribe `/image_raw/compressed` (QoS: Best Effort)
- Giải nén JPEG → chạy YOLO inference
- Nếu phát hiện lửa → publish `/fire_target` (geometry_msgs/Point): tọa độ tâm lửa chuẩn hóa [-1, 1]
- Publish ảnh đã vẽ bounding box lên `/yolo/image` (để Web Dashboard hiển thị)

**🧪 Debug 4.2:**
```
✅ colcon build → pass
✅ ros2 launch fire_robot_perception perception.launch.py
✅ Đưa lửa trước camera → ros2 topic echo /fire_target → có tọa độ Point
✅ rqt_image_view chọn /yolo/image → thấy bounding box đỏ quanh lửa
✅ Bỏ lửa đi → /fire_target ngưng publish
✅ FPS YOLO ≥ 5 (tùy GPU)
```

---

### Bước 4.3: Fire Logic Node (Điều khiển bơm)
**Làm gì:** Viết `fire_logic.py`.
- Subscribe `/fire_target` + `/env_status` (cảm biến IR từ WROOM)
- Logic: Nếu YOLO phát hiện lửa VÀ confidence > threshold → publish `/pump_cmd` = True
- Nếu hết lửa > 3 giây → publish `/pump_cmd` = False
- Log đầy đủ quyết định

**🧪 Debug 4.3:**
```
✅ Lửa trước cam → /pump_cmd = True → ESP32 bật bơm
✅ Bỏ lửa > 3s → /pump_cmd = False → ESP32 tắt bơm
✅ Confidence thấp (lửa xa) → không bật bơm (tránh false positive)
```

> [!TIP]
> **Checkpoint Phase 4:** YOLO phát hiện lửa, tự động điều khiển servo + bơm.

---

## PHASE 5: WEB DASHBOARD

### Bước 5.1: Tạo package fire_robot_teleop
**Làm gì:** `ros2 pkg create fire_robot_teleop --build-type ament_python`
- Tạo thư mục `web/` chứa HTML/CSS/JS

**🧪 Debug 5.1:**
```
✅ colcon build → pass
```

---

### Bước 5.2: Rosbridge WebSocket
**Làm gì:** Thêm `rosbridge_server` vào `dashboard.launch.py`.
- Cài: `sudo apt install ros-humble-rosbridge-server`
- Cấu hình port 9090

**🧪 Debug 5.2:**
```
✅ ros2 launch fire_robot_bringup dashboard.launch.py
✅ Laptop browser: mở ws://10.0.0.1:9090 → WebSocket connected
```

---

### Bước 5.3: Web Dashboard Frontend
**Làm gì:** Viết `index.html` + JS modules.
- Camera Feed: subscribe `/image_raw/compressed` qua roslibjs
- Sensor Panel: subscribe `/env_status` → hiển thị Gas, Temp, Batt, Fire
- Teleop: Joystick ảo → publish `/cmd_vel`
- Robot Status: subscribe `/odom` → hiển thị tọa độ + trạng thái

**🧪 Debug 5.3:**
```
✅ Mở http://10.0.0.1:8080 → trang web load thành công
✅ Camera feed hiển thị ≥ 10 FPS
✅ Sensor data cập nhật mỗi 1 giây
✅ Nhấn nút Tiến → xe chạy tiến, Dừng → xe dừng
✅ Joystick ảo hoạt động mượt
```

---

### Bước 5.4: YOLO Overlay trên Web
**Làm gì:** Khi YOLO phát hiện lửa, hiển thị bounding box + cảnh báo trên camera feed.

**🧪 Debug 5.4:**
```
✅ Lửa trước cam → Web Dashboard hiện khung đỏ + "🔥 FIRE DETECTED (87%)"
✅ Alert panel nhấp nháy cảnh báo
✅ Bỏ lửa → cảnh báo tắt
```

> [!TIP]
> **Checkpoint Phase 5:** Web Dashboard đầy đủ chức năng giám sát + điều khiển.

---

## PHASE 6: TÍCH HỢP TOÀN BỘ + STRESS TEST

### Bước 6.1: Full System Test
```
✅ Pi: ros2 launch fire_robot_bringup robot.launch.py → tất cả node khởi chạy
✅ Laptop: ros2 launch fire_robot_bringup station.launch.py → SLAM + Nav2 + YOLO
✅ RViz hiển thị bản đồ + xe + Lidar + camera
✅ Web Dashboard hiển thị đầy đủ
```

### Bước 6.2: Full Flow chữa cháy
```
✅ Đặt lửa trong phòng → YOLO phát hiện → /fire_target publish
✅ Nav2 điều hướng xe đến gần lửa
✅ fire_logic bật bơm → ESP32 → WROOM → Relay bơm + Servo xoay
✅ Dập lửa xong → bơm tắt
✅ Latency end-to-end (cam→YOLO→servo) < 500ms
```

### Bước 6.3: Fail-Safe Test
```
✅ Đang chạy Nav2 → tắt WiFi Laptop → Pi watchdog gửi stop (< 500ms) → xe dừng
✅ Đang chạy → rút USB ESP32 → ESP32 EMERGENCY (< 1s) → xe dừng + buzzer
✅ Kết nối lại WiFi → ROS 2 re-discovery < 3s → tiếp tục hoạt động
✅ Kết nối lại USB → micro-ROS reconnect → xe về IDLE
```

### Bước 6.4: Stress Test 30 phút
```
✅ Hệ thống chạy liên tục 30 phút: Nav2 + YOLO + Dashboard
✅ CPU Pi < 70% (htop)
✅ Không crash, không memory leak
✅ Camera ≥ 10 FPS liên tục
✅ Lidar /scan ≥ 10 Hz liên tục
✅ Odometry liên tục, không mất gói
```

---

## 📊 TỔNG KẾT

| Phase | Số bước | Thời gian ước tính |
|-------|---------|-------------------|
| Phase 1: URDF + TF | 3 bước | 1 ngày |
| Phase 2: Sensors Pi | 6 bước | 3-4 ngày |
| Phase 3: SLAM + Nav2 | 4 bước | 3-5 ngày |
| Phase 4: YOLO | 3 bước | 3-4 ngày |
| Phase 5: Web Dashboard | 4 bước | 3-4 ngày |
| Phase 6: Tích hợp | 4 bước | 2-3 ngày |
| **Tổng** | **24 bước** | **~15-21 ngày** |
