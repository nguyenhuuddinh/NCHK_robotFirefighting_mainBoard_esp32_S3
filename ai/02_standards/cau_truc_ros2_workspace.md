# Cấu trúc thư mục cho ROS 2 Workspace (Pi + Laptop)

```text
ros2_ws/
├── src/
│   ├── fire_robot_description/        # Mô tả vật lý robot (URDF + TF)
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── urdf/
│   │   │   └── fire_robot.urdf.xacro  # Mô hình xe: base_link, laser_frame, camera_frame, imu_frame
│   │   ├── meshes/                    # File 3D (nếu có) cho RViz
│   │   ├── launch/
│   │   │   └── description.launch.py  # Khởi chạy robot_state_publisher + joint_state_publisher
│   │   └── config/
│   │       └── rviz_config.rviz       # Cấu hình RViz mặc định
│   │
│   ├── fire_robot_bringup/            # Launch files tổng hợp (điểm vào chính)
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── launch/
│   │   │   ├── robot.launch.py        # 🟢 CHẠY TRÊN PI: Lidar + Camera + micro-ROS Agent + safety_watchdog + rosbridge
│   │   │   ├── station.launch.py      # 🔵 CHẠY TRÊN LAPTOP: SLAM + Nav2 + YOLO + RViz
│   │   │   ├── sensors.launch.py      # Sub-launch: rplidar + usb_cam
│   │   │   ├── micro_ros.launch.py    # Sub-launch: micro_ros_agent
│   │   │   └── dashboard.launch.py    # Sub-launch: rosbridge_server
│   │   └── config/
│   │       ├── pi_params.yaml         # Tham số cho Pi (camera resolution, lidar port...)
│   │       └── network_params.yaml    # IP, DOMAIN_ID, DDS config
│   │
│   ├── fire_robot_navigation/         # SLAM + Nav2 (chạy trên Laptop)
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── launch/
│   │   │   ├── slam.launch.py         # Khởi chạy slam_toolbox (online async)
│   │   │   ├── nav2.launch.py         # Khởi chạy Nav2 stack
│   │   │   └── localization.launch.py # Khởi chạy AMCL (dùng bản đồ đã lưu)
│   │   ├── config/
│   │   │   ├── slam_params.yaml       # Tham số slam_toolbox
│   │   │   ├── nav2_params.yaml       # Tham số Nav2 (planner, controller, costmap)
│   │   │   └── amcl_params.yaml       # Tham số AMCL localization
│   │   └── maps/                      # Thư mục lưu bản đồ đã SLAM
│   │       └── .gitkeep
│   │
│   ├── fire_robot_perception/         # YOLO nhận diện lửa (chạy trên Laptop)
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── fire_robot_perception/     # Python package
│   │   │   ├── __init__.py
│   │   │   ├── yolo_fire_detector.py  # Node: subscribe /image_raw/compressed → YOLO → publish /fire_target
│   │   │   └── fire_logic.py          # Node: logic quyết định bơm nước (subscribe /fire_target → publish /pump_cmd)
│   │   ├── config/
│   │   │   └── yolo_params.yaml       # Ngưỡng confidence, model path, input size
│   │   ├── models/                    # File weight YOLO (.pt hoặc .onnx)
│   │   │   └── .gitkeep
│   │   └── launch/
│   │       └── perception.launch.py
│   │
│   ├── fire_robot_safety/             # Node an toàn (chạy trên Pi)
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── fire_robot_safety/
│   │   │   ├── __init__.py
│   │   │   └── safety_watchdog.py     # Node: giám sát heartbeat /cmd_vel, gửi stop nếu mất > 500ms
│   │   └── launch/
│   │       └── safety.launch.py
│   │
│   └── fire_robot_teleop/             # Web Dashboard + Teleop (frontend chạy trên browser Laptop)
│       ├── package.xml
│       ├── setup.py
│       ├── fire_robot_teleop/
│       │   ├── __init__.py
│       │   └── web_server.py          # Node: HTTP server phục vụ trang web dashboard
│       ├── web/                       # Frontend HTML/CSS/JS
│       │   ├── index.html             # Trang chính Dashboard
│       │   ├── css/
│       │   │   └── style.css
│       │   └── js/
│       │       ├── app.js             # Logic chính
│       │       ├── roslibjs.min.js    # Thư viện kết nối WebSocket ROS 2
│       │       ├── teleop.js          # Joystick ảo → /cmd_vel
│       │       ├── camera.js          # Hiển thị camera feed
│       │       └── sensors.js         # Hiển thị sensor data
│       └── launch/
│           └── teleop.launch.py
│
├── .gitignore                         # Ignore: build/, install/, log/
└── README.md                          # Hướng dẫn build + chạy
```

## Phân biệt: Chạy ở đâu?

| Package | Chạy trên Pi | Chạy trên Laptop | Ghi chú |
|:---|:---:|:---:|:---|
| `fire_robot_description` | ✅ | ✅ | Cả 2 cần TF tree |
| `fire_robot_bringup` | ✅ (`robot.launch.py`) | ✅ (`station.launch.py`) | Launch file khác nhau |
| `fire_robot_navigation` | ❌ | ✅ | SLAM + Nav2 chỉ chạy trên Laptop |
| `fire_robot_perception` | ❌ | ✅ | YOLO cần GPU Laptop |
| `fire_robot_safety` | ✅ | ❌ | Watchdog phải chạy trên Pi |
| `fire_robot_teleop` | ✅ (host web) | ❌ (mở browser) | Web server trên Pi, browser trên Laptop |

## So sánh với bản ESP32

| ESP32 (PlatformIO) | ROS 2 Workspace |
|:---|:---|
| `lib/common/` — Config, Structs | `*/config/*.yaml` — Parameters |
| `lib/driver/` — Hardware access | `rplidar_ros`, `usb_cam` — Sensor drivers |
| `lib/service/` — Algorithms | `fire_robot_navigation/`, `fire_robot_perception/` |
| `lib/app/` — State machine, tasks | `fire_robot_bringup/`, `fire_robot_safety/` |
| `src/main.cpp` — Entry point | `*/launch/*.launch.py` — Entry points |
