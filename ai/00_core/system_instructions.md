# AI System Instructions - Dự án NCKH Robot Chữa Cháy Tự Hành

Đây là "Bản Hiến Pháp" định hình tư duy, cách suy nghĩ và hành động của AI khi làm việc trong dự án này. Mọi AI Agent (Vai trò) được gọi lên đều phải tuân thủ nghiêm ngặt các nguyên tắc dưới đây.

---

## 1. Tổng quan Dự án

### 1.1. Mục tiêu
Robot chữa cháy tự hành sử dụng SLAM (vẽ bản đồ), Nav2 (tự lái), YOLO (nhận diện lửa) qua mạng WiFi nội bộ do Raspberry Pi phát.

### 1.2. Kiến trúc Phần cứng (4 thành phần)
| Thành phần | MCU | Nhiệm vụ chính |
|:---|:---|:---|
| **Laptop** | Ubuntu + ROS 2 | SLAM, Nav2, YOLO, Web Dashboard |
| **Raspberry Pi** | Ubuntu + ROS 2 | WiFi AP, Lidar, Camera, micro-ROS Agent, rosbridge |
| **ESP32-S3** | FreeRTOS | 4 Motor + 4 Encoder + IMU + PID + Odometry + micro-ROS |
| **ESP32-WROOM** | FreeRTOS | Sensor (lửa IR, gas, nhiệt độ, pin) + Actuator (servo pan, relay bơm, buzzer) |

### 1.3. Giao tiếp
- Laptop ↔ Pi: **WiFi AP** (Pi phát, Laptop kết nối) + ROS 2 DDS Multicast
- Pi ↔ ESP32-S3: **USB Serial** (micro-ROS)
- ESP32-S3 ↔ ESP32-WROOM: **UART** (SensorPacket_t / ActuatorCmd_t + CRC8)

---

## 2. Triết lý Phát triển (Builder Ethos)

### 2.1. Boil the Lake (Làm trọn vẹn)
Chi phí để viết một module hoàn chỉnh (bao gồm cả xử lý lỗi, logging, và tối ưu) là gần bằng không.
- **Không code cắt xén:** Không để lại `// TODO: Thêm logic tại đây`. Nếu thiếu thông số, dùng `#define` với giá trị giả định hợp lý và triển khai logic hoàn chỉnh.
- **Biển và Hồ:** Luôn hoàn thiện 100% "Hồ" (xử lý triệt để 1 module). Tránh bơi ra "Biển" (tự ý đề xuất đập bỏ toàn bộ kiến trúc).

### 2.2. Search Before Building (Tìm trước khi tự code)
Embedded rất nhạy cảm với tài nguyên:
1. Luôn kiểm tra ESP-IDF hoặc thư viện chuẩn C++ đã có sẵn hàm tối ưu chưa.
2. Khi làm micro-ROS, ưu tiên message type chuẩn ROS 2 thay vì tự định nghĩa struct.
3. Khi cần thuật toán (PID, Kalman, EMA...), tìm hiểu cách triển khai chuẩn trước khi tự viết.

### 2.3. User Sovereignty (Người dùng là tối thượng)
- **Đề xuất, không tự ý thay đổi:** Nếu thấy hướng tiếp cận tốt hơn, hãy ĐỀ XUẤT và chờ đồng ý. Không tự xóa file cũ.
- **Hỏi trước khi phá kiến trúc:** Ví dụ: "Thêm ngắt vào chân này có thể xung đột I2C. Bạn muốn tiếp tục không?"
- **Không thêm dependency không cần thiết:** Mỗi thư viện thêm vào đều tăng thời gian build và chiếm Flash/RAM.

---

## 3. Tiêu chuẩn Mã Nguồn (Code Quality)

### 3.1. Nguyên tắc bắt buộc
- **Không che giấu lỗi:** Không dùng `try/catch` rỗng. Nếu I2C init fail → Log lỗi rõ ràng, fallback hoặc dừng.
- **Không Blocking:** Cấm `delay()` trong `lib/`. Dùng `vTaskDelay()` hoặc `millis()`.
- **Quản lý Bộ nhớ:** Cấm `malloc/new/String` trong vòng lặp vô hạn. Ưu tiên cấp phát tĩnh.
- **Mutex bắt buộc:** Khi >1 Task truy cập cùng biến → phải dùng `Semaphore/Mutex`.

### 3.2. Quy ước đặt tên
- **File:** PascalCase (`MotorDriver.cpp`, `PIDController.h`)
- **Class:** PascalCase (`RobotMaster`, `SlaveComm`)
- **Hàm:** camelCase (`computeWheelSpeeds`, `getGyroZ`)
- **Hằng số:** UPPER_SNAKE_CASE (`WHEEL_RADIUS_M`, `PID_KP`)
- **Biến global shared:** prefix `g_` (`g_target_vx`, `g_stateMutex`)

### 3.3. Logging chuẩn
Mọi log Serial phải có prefix rõ ràng để dễ filter:
```
[OK]    Khởi tạo thành công
[WARN]  Cảnh báo (fallback)
[ERROR] Lỗi nghiêm trọng
[CMD]   Nhận lệnh
[ODOM]  Dữ liệu odometry
[PID]   Dữ liệu tuning
[UART]  Giao tiếp WROOM
[uROS]  micro-ROS
```

---

## 4. Kiến trúc Thư mục Dự án

```text
NCHK_robotFirefighting_mainBoard_esp32_S3/
├── ai/                     # Luật lệ và vai trò cho AI (CHỈ ĐỌC, KHÔNG SỬA)
│   ├── 00_core/            # Bản hiến pháp (file này)
│   ├── 01_roles/           # Vai trò: firmware_engineer, qa_engineer
│   ├── 02_standards/       # Tài liệu chuẩn: kiến trúc, pinout, giao thức
│   └── 03_templates/       # Template: bug report, tuning log
├── lib/                    # Mã nguồn chia theo Layer
│   ├── common/             # DataStructs.h, PinConfig.h, RobotConfig.h
│   ├── driver/             # EncoderDriver, MotorDriver, IMUDriver, SlaveComm
│   ├── service/            # PIDController, Kinematics, Odometry, MicroRosComm
│   └── app/                # RobotMaster, TaskManager, ActuatorLogic
├── src/main.cpp            # Entry point
└── platformio.ini          # Cấu hình PlatformIO
```

### Quy tắc Layer:
- `driver/` → CHỈ giao tiếp phần cứng. Không chứa logic nghiệp vụ.
- `service/` → Thuật toán thuần (PID, Kinematics). Không truy cập GPIO trực tiếp.
- `app/` → Logic ứng dụng (state machine, task management). Gọi driver + service.
- `common/` → Struct, pin mapping, config. Không chứa code thực thi.
- **Không import ngược:** `driver/` KHÔNG import từ `service/` hay `app/`.

---

## 5. Quy trình Làm việc Bắt buộc

### 5.1. Trước khi code
1. Đọc file vai trò (role) tương ứng trong `ai/01_roles/`.
2. Đọc tài liệu chuẩn liên quan trong `ai/02_standards/`.
3. Kiểm tra code hiện tại — HIỂU trước khi sửa.

### 5.2. Khi code
1. Mỗi thay đổi phải có MỤC TIÊU rõ ràng và CÁCH VERIFY.
2. Không sửa nhiều module cùng lúc — sửa 1, test 1, rồi mới tiếp.
3. Giữ nguyên comment/docstring không liên quan đến thay đổi.

### 5.3. Sau khi code
1. Liệt kê: file nào đã sửa, dòng nào, tại sao.
2. Đề xuất cách test/verify cho người dùng.
3. Cảnh báo side-effect nếu có (ví dụ: "Sửa IMUDriver có thể ảnh hưởng Odometry").