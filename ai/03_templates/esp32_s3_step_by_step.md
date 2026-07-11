# 🔧 QUY TRÌNH XÂY DỰNG ESP32-S3 — TỪNG BƯỚC NHỎ + DEBUG

> [!IMPORTANT]
> **Nguyên tắc vàng:** Mỗi bước phải **build thành công + chạy đúng** trước khi qua bước tiếp. Nếu sai → fix ngay tại bước đó, KHÔNG tiến tiếp.

---

## PHASE 1: REFACTOR CẤU TRÚC (Tách main.cpp monolithic)

### Bước 1.1: Cập nhật DataStructs.h
**Làm gì:** Sửa enum `RobotState_t` và struct giao tiếp WROOM cho đúng spec mới.
- Đổi `RobotState_t`: `STATE_AUTO_NAV, STATE_OVERRIDE, STATE_EMERGENCY` → `STATE_IDLE, STATE_AUTO, STATE_MANUAL, STATE_EMERGENCY`
- Thay `WroomPacket_t` (cũ, dùng joystick) → `SensorPacket_t` (14 bytes: header + fire + gas + temp + batt + crc)
- Thêm `ActuatorCmd_t` (5 bytes: header + servo_pan + pump + buzzer + crc)

**🧪 Debug 1.1:**
```
✅ Build pass (pio run) — không lỗi compile
✅ Các file khác dùng RobotState_t cũ (nếu có) → sửa theo
✅ sizeof(SensorPacket_t) == 14, sizeof(ActuatorCmd_t) == 5
```

---

### Bước 1.2: Tạo RobotMaster — State Machine
**Làm gì:** Implement class quản lý 4 trạng thái robot.
- Biến `currentState` (mặc định IDLE)
- Hàm `setState()`, `getState()` — thread-safe với atomic
- Hàm `update()`: Logic chuyển state dựa trên kết nối micro-ROS + timeout
- Tạm thời: STATE_IDLE → STATE_MANUAL (khi có Web cmd) → STATE_IDLE (khi hết cmd)

**🧪 Debug 1.2:**
```
✅ Build pass
✅ Thêm vào main.cpp: RobotMaster master; master.init();
✅ Serial log: "[STATE] IDLE → MANUAL" khi gửi lệnh từ Web
✅ Serial log: "[STATE] MANUAL → IDLE" khi dừng lệnh 2 giây
✅ Xe vẫn chạy bình thường như trước (không thay đổi motion logic)
```

---

### Bước 1.3: Tách Feed-Forward params ra khỏi main.cpp
**Làm gì:** Di chuyển các hằng số PWM_OFFSET, K_FF, TUNING_STEP logic vào `RobotConfig.h`.
- Move `PWM_OFFSET`, `PWM_OFFSET_TURN`, `K_FF`, `PWM_MAX` → `RobotConfig.h`
- Move logic `#if TUNING_STEP == 1/2/3/4` → `RobotConfig.h` (export `KP_ACT, KI_ACT, KD_ACT, K_FF_ACT`)
- main.cpp chỉ `#include "RobotConfig.h"` và dùng giá trị

**🧪 Debug 1.3:**
```
✅ Build pass
✅ Serial log "[CFG]" in ra giá trị giống hệt trước refactor
✅ Xe chạy PID giống hệt (so sánh log Tgt/Act/Err/PWM)
```

---

### Bước 1.4: Tách hàm feedForward() và computePWM() ra file service
**Làm gì:** Tạo file `lib/service/MotionControl.h/.cpp` chứa logic tính PWM.
- `feedForward(omega_tgt, offset_pwm) → float`
- `computePWM(omega_tgt, omega_act, pid, offset) → float`
- `applyRamp(current, target, max_rate, dt) → float`
- main.cpp gọi hàm thay vì define inline

**🧪 Debug 1.4:**
```
✅ Build pass
✅ Log PWM 4 bánh giống hệt trước khi tách
✅ Chạy TIEN/LUI/TRAI/PHAI — hành vi xe không đổi
```

---

### Bước 1.5: Tách Task_Motion ra file riêng
**Làm gì:** Di chuyển hàm `Task_Motion()` từ main.cpp → `lib/app/TaskMotion.h/.cpp`
- File chứa: hàm `void Task_Motion(void* pvParam)`
- Include các driver + service cần thiết
- Shared data (g_cmdMutex, g_target_vx...) → truyền qua struct context hoặc extern

**🧪 Debug 1.5:**
```
✅ Build pass
✅ Xe chạy giống hệt — so sánh log [FL][RL][FR][RR] và [ODOM]
✅ Task chạy đúng Core 1, priority 5
```

---

### Bước 1.6: Tách Task_Logger ra file riêng
**Làm gì:** Di chuyển `Task_Logger()` → `lib/app/TaskLogger.h/.cpp`

**🧪 Debug 1.6:**
```
✅ Build pass
✅ Serial log vẫn hiện đầy đủ khi xe chạy
```

---

### Bước 1.7: Tách Web Server ra module riêng (giữ tạm)
**Làm gì:** Di chuyển WebServer + HTML + handleCmd/handleNav → `lib/app/WebControl.h/.cpp`
- `WebControl::init(SemaphoreHandle_t cmdMutex)` — setup routes
- `WebControl::handleClient()` — gọi trong Task_WebServer
- HTML page là `static const char*` trong .cpp

**🧪 Debug 1.7:**
```
✅ Build pass
✅ Mở 192.168.4.1 trên điện thoại → giao diện hiện bình thường
✅ Nhấn TIEN/LUI/TRAI/PHAI → xe chạy đúng
✅ Auto Nav → xe tự đi đến tọa độ
```

---

### Bước 1.8: Implement TaskManager
**Làm gì:** `TaskManager::init()` tập trung tạo tất cả FreeRTOS tasks.
- Tạo Mutex (g_cmdMutex, g_stateMutex)
- `xTaskCreatePinnedToCore` cho Task_Motion, Task_WebServer, Task_Logger
- main.cpp `setup()` chỉ còn: `Serial.begin() → driver.init() → TaskManager::init()`

**🧪 Debug 1.8:**
```
✅ Build pass
✅ main.cpp < 50 dòng
✅ Xe chạy giống hệt bản gốc 568 dòng
✅ Tất cả test trước đó vẫn pass
```

> [!TIP]
> **Checkpoint Phase 1 hoàn thành:** main.cpp gọn < 50 dòng, code modular, xe chạy 100% giống bản gốc.

---

## PHASE 2: GIAO TIẾP UART S3 ↔ WROOM

### Bước 2.1: SlaveComm — Init UART2
**Làm gì:** Implement `SlaveComm::init()` — khởi tạo UART2 trên GPIO 19/20.
- Baudrate 115200, 8N1
- Dùng `HardwareSerial Serial2` hoặc ESP-IDF `uart_driver_install`
- Chỉ init, chưa gửi/nhận gì

**🧪 Debug 2.1:**
```
✅ Build pass
✅ Serial log: "[UART2] Initialized on GPIO 19/20, 115200 baud"
✅ Dùng oscilloscope/logic analyzer kiểm tra TX pin có tín hiệu khi gửi test byte
```

---

### Bước 2.2: Implement CRC8
**Làm gì:** Viết hàm `uint8_t calcCRC8(const uint8_t* data, size_t len)`.
- Dùng polynomial 0x07 (CRC-8 chuẩn)
- Hàm nhỏ, dùng chung cho cả send và receive

**🧪 Debug 2.2:**
```
✅ Build pass
✅ Test vector: calcCRC8({0xAA, 0x01, 0x00, 0x00, ...}, 13) == giá trị đúng
✅ Chạy 3-4 test case khác nhau, so sánh với CRC online calculator
```

---

### Bước 2.3: SlaveComm — Parse SensorPacket_t (nhận từ WROOM)
**Làm gì:** Implement `bool SlaveComm::tryReceive(SensorPacket_t& out)`.
- Đọc byte từ UART2 buffer
- Tìm header 0xAA → đọc tiếp 13 bytes
- Verify CRC8 → nếu pass thì copy vào `out`, return true
- Nếu CRC fail → discard, return false

**🧪 Debug 2.3 (KHÔNG CẦN WROOM thật):**
```
✅ Build pass
✅ Test bằng cách nối TX↔RX (loopback) trên S3:
   - S3 tự gửi 1 SensorPacket_t mẫu qua Serial2.write()
   - Rồi gọi tryReceive() → phải parse ra đúng giá trị
✅ Test gửi packet lỗi (sai CRC) → tryReceive() return false
✅ Test gửi packet thiếu byte → không crash, return false
```

---

### Bước 2.4: SlaveComm — Send ActuatorCmd_t (gửi xuống WROOM)
**Làm gì:** Implement `void SlaveComm::sendCommand(uint8_t servoPan, bool pumpOn, bool buzzerOn)`.
- Đóng gói: header 0xBB + servo_pan + pump + buzzer + CRC8
- Gửi qua Serial2.write()

**🧪 Debug 2.4 (loopback test):**
```
✅ Build pass
✅ Gửi lệnh → đọc lại qua loopback → parse header 0xBB + kiểm tra CRC
✅ Gửi servo_pan=90, pump=1, buzzer=0 → verify từng byte
```

---

### Bước 2.5: Tạo Task_UART_Rx trên Core 0
**Làm gì:** FreeRTOS task nhận data từ WROOM liên tục.
- Chu kỳ 50ms
- Gọi `slaveComm.tryReceive()` → nếu có data → `xQueueOverwrite(g_sensorQueue, &packet)`
- Đếm CRC pass/fail để đánh giá chất lượng UART

**🧪 Debug 2.5:**
```
✅ Build pass
✅ Chạy 1 phút → Serial log đếm: "UART packets: 600 ok, 2 crc_fail (99.7%)"
✅ Nếu chưa có WROOM thật → dùng USB-TTL converter + PC gửi packet giả lập
✅ Task không làm crash các task khác (Motion, Logger vẫn chạy bình thường)
```

---

### Bước 2.6: Tích hợp sensor data vào Logger
**Làm gì:** Task_Logger thêm dòng log hiển thị dữ liệu cảm biến từ WROOM.

**🧪 Debug 2.6:**
```
✅ Serial log: "[ENV] Fire:001 Gas:120.5ppm Temp:32.1°C Batt:11.8V"
✅ Giá trị thay đổi realtime khi WROOM gửi data khác
```

> [!TIP]
> **Checkpoint Phase 2:** S3 ↔ WROOM giao tiếp 2 chiều, CRC > 99%, xe vẫn chạy motion bình thường.

---

## PHASE 3: CẢI THIỆN ODOMETRY

### Bước 3.1: Adaptive Complementary Filter
**Làm gì:** Sửa `Odometry::update()` — α thay đổi theo tình huống.
- `|ω_gyro| > 0.1 rad/s` → α = 0.995 (tin gyro vì encoder trượt)
- `|ω_gyro| ≤ 0.1 rad/s` → α = 0.95 (blend đều)

**🧪 Debug 3.1:**
```
✅ Build pass
✅ Log thêm giá trị α: "[ODOM] alpha=0.995" khi xoay, "alpha=0.950" khi thẳng
✅ Xe đi thẳng 1m → odom X thay đổi, Y gần 0 (< 3cm)
✅ Xe xoay 360° tại chỗ → odom theta quay về gần 0 (< 15°)
```

---

### Bước 3.2: Calibrate WHEEL_RADIUS_M
**Làm gì:** Đo thực nghiệm bán kính bánh xe.
- Đánh dấu sàn → cho xe đi thẳng 1m → đọc odom X
- Nếu odom X > 1m → giảm WHEEL_RADIUS_M, ngược lại tăng
- Lặp cho đến sai số < 5%

**🧪 Debug 3.2:**
```
✅ 3 lần đi thẳng 1m: odom X = 0.97, 1.02, 0.99 → sai số < 5% ✓
✅ 3 lần lùi 1m: odom X thay đổi tương ứng, sai số < 5%
```

---

### Bước 3.3: Calibrate TRACK_WIDTH_M
**Làm gì:** Đo thực nghiệm khoảng cách trục bánh.
- Cho xe xoay 360° tại chỗ → đọc odom theta
- Nếu theta > 2π → tăng TRACK_WIDTH_M, ngược lại giảm
- Lặp cho đến sai số < 5%

**🧪 Debug 3.3:**
```
✅ Xoay 360° CW: theta ≈ 6.28 rad (±0.3)
✅ Xoay 360° CCW: theta ≈ -6.28 rad (±0.3)
✅ Xoay 720° (2 vòng): theta ≈ 12.56 rad (±0.6)
```

---

### Bước 3.4: Test hình vuông 1m × 1m
**Làm gì:** Điều khiển xe đi hình vuông, 4 cạnh × 1m, quay 90° tại mỗi góc.

**🧪 Debug 3.4:**
```
✅ Vị trí cuối (odom X, Y) gần vị trí đầu — sai số < 10% (< 40cm sau 4m)
✅ Theta cuối gần 0 (hoặc 2π) — sai số < 20°
✅ Log đầy đủ trajectory để phân tích
```

> [!TIP]
> **Checkpoint Phase 3:** Odometry chính xác, sẵn sàng cho SLAM trên Pi.

---

## PHASE 4: MICRO-ROS INTEGRATION

### Bước 4.0: Giải quyết vấn đề đường dẫn
**Làm gì:** Tạo symlink hoặc di chuyển project để loại bỏ khoảng trắng trong path.

**🧪 Debug 4.0:**
```
✅ pio run build thành công với micro_ros_platformio trong lib_deps
```

---

### Bước 4.1: Thêm micro-ROS lib + build thử
**Làm gì:** Uncomment `micro_ros_platformio` trong `platformio.ini`, thêm build flags.

**🧪 Debug 4.1:**
```
✅ pio run build pass (có thể mất 5-10 phút lần đầu)
✅ Flash lên ESP32-S3 không crash
✅ Serial log: "micro-ROS lib loaded"
```

---

### Bước 4.2: MicroRosComm — Init transport + Node
**Làm gì:** Implement `MicroRosComm::init()`.
- USB Serial transport (`set_microros_serial_transports(Serial)`)
- Tạo Node: `motion_slave`
- Tạo Executor
- **Chưa tạo publisher/subscriber**

**🧪 Debug 4.2:**
```
✅ Build pass
✅ Chạy micro-ROS Agent trên Pi: "ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0"
✅ Agent log: "Node 'motion_slave' connected"
✅ ros2 node list → thấy /motion_slave
```

---

### Bước 4.3: Publisher /odom
**Làm gì:** Tạo publisher cho nav_msgs/Odometry.
- Đọc data từ g_odometry (mutex) → fill message → publish
- Tần suất 50Hz (trong Task_MicroROS)

**🧪 Debug 4.3:**
```
✅ Build pass
✅ Trên Pi: ros2 topic echo /odom → thấy position.x, y, orientation thay đổi
✅ Cho xe đi thẳng → /odom position.x tăng dần
✅ Cho xe xoay → /odom orientation thay đổi
✅ Hz check: ros2 topic hz /odom → ~50Hz
```

---

### Bước 4.4: Publisher /imu/data
**Làm gì:** Tạo publisher sensor_msgs/Imu.

**🧪 Debug 4.4:**
```
✅ ros2 topic echo /imu/data → gyro_z, accel_x/y/z có giá trị hợp lý
✅ Lắc robot → giá trị thay đổi realtime
```

---

### Bước 4.5: Publisher /env_status
**Làm gì:** Tạo publisher std_msgs/String, nội dung JSON từ WROOM sensor data.

**🧪 Debug 4.5:**
```
✅ ros2 topic echo /env_status → JSON: {"fire":"001","gas":120.5,"temp":32.1,"batt":11.8}
```

---

### Bước 4.6: TF broadcaster (odom → base_link)
**Làm gì:** Publish transform tree bắt buộc cho SLAM.

**🧪 Debug 4.6:**
```
✅ Trên Pi: ros2 run tf2_ros tf2_echo odom base_link → thấy translation + rotation
✅ RViz hiển thị TF tree đúng
```

---

### Bước 4.7: Subscriber /cmd_vel
**Làm gì:** Subscribe geometry_msgs/Twist → ghi vào g_target_vx, g_target_wz.

**🧪 Debug 4.7:**
```
✅ Build pass
✅ Trên Pi: ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" → XE CHẠY TIẾN
✅ Pub angular z: 2.0 → xe xoay
✅ Pub linear 0 + angular 0 → xe dừng
✅ Dùng teleop_twist_keyboard → điều khiển xe bình thường
```

---

### Bước 4.8: Subscriber /fire_target + /pump_cmd
**Làm gì:** Subscribe → chuyển tiếp qua UART xuống WROOM.

**🧪 Debug 4.8:**
```
✅ Pub /fire_target Point(x=0.5) → Serial log: "[ACT] Servo Pan → 135°"
✅ Pub /pump_cmd Bool(true) → Serial log: "[ACT] Pump ON"
✅ Verify WROOM nhận đúng ActuatorCmd_t qua UART
```

---

### Bước 4.9: Watchdog — Emergency khi mất kết nối
**Làm gì:** Nếu không nhận /cmd_vel > 1000ms → STATE_EMERGENCY → PWM=0 + Buzzer.

**🧪 Debug 4.9:**
```
✅ Đang chạy xe → rút USB → xe dừng trong < 1 giây
✅ Serial log (nếu còn): "[STATE] AUTO → EMERGENCY"
✅ Cắm USB lại → Agent reconnect → "[STATE] EMERGENCY → IDLE"
✅ Gửi /cmd_vel lại → xe chạy tiếp bình thường
```

---

### Bước 4.10: Xóa Web Server + WiFi AP
**Làm gì:** Remove toàn bộ WebControl, WiFi.softAP, WebServer khỏi project.
- Xóa file `WebControl.h/.cpp`
- Xóa `#include <WebServer.h>` và `#include <WiFi.h>`
- UART0 (USB) giờ dành 100% cho micro-ROS

**🧪 Debug 4.10:**
```
✅ Build pass — kích thước firmware giảm đáng kể
✅ RAM free tăng (Serial log heap size)
✅ Toàn bộ test từ 4.3-4.9 vẫn pass
✅ WiFi scan không thấy "ESP32_Robot" nữa
```

> [!TIP]
> **Checkpoint Phase 4:** ESP32-S3 là ROS 2 Node hoàn chỉnh, không còn Web Server.

---

## PHASE 5: TÍCH HỢP TOÀN BỘ + STRESS TEST

### Bước 5.1: Full State Machine test
```
✅ Boot → IDLE (PWM=0)
✅ Agent connect → IDLE (chờ lệnh)
✅ /cmd_vel đến → AUTO hoặc MANUAL
✅ Mất kết nối > 1s → EMERGENCY (phanh + buzzer)
✅ Reconnect → IDLE
✅ Cycle 10 lần: không crash
```

### Bước 5.2: Full flow YOLO → chữa cháy
```
✅ Laptop pub /fire_target → S3 nhận → tính servo → UART → WROOM → Servo xoay
✅ Laptop pub /pump_cmd true → S3 → UART → WROOM → Relay bật bơm
✅ Laptop pub /pump_cmd false → bơm tắt
✅ Latency < 100ms (đo từ pub → servo phản hồi)
```

### Bước 5.3: Stress test 30 phút
```
✅ Xe chạy teleop liên tục 30 phút
✅ Không crash, không memory leak (heap free ổn định)
✅ UART CRC pass > 99%
✅ /odom publish liên tục, không mất gói
✅ Nhiệt độ ESP32-S3 < 70°C
```

### Bước 5.4: EMI test (nhiễu bơm nước)
```
✅ Bật/tắt bơm nước 20 lần liên tiếp
✅ UART không bị lỗi CRC đột biến
✅ I2C (IMU) không bị treo
✅ Encoder count không bị nhảy giá trị bất thường
```

---

## 📊 TỔNG KẾT: 24 BƯỚC + 24 DEBUG CHECKPOINT

| Phase | Số bước | Thời gian ước tính |
|-------|---------|-------------------|
| Phase 1: Refactor | 8 bước | 2-3 ngày |
| Phase 2: UART WROOM | 6 bước | 3 ngày |
| Phase 3: Odometry | 4 bước | 2-3 ngày |
| Phase 4: micro-ROS | 11 bước | 5-7 ngày |
| Phase 5: Tích hợp | 4 bước | 2 ngày |
| **Tổng** | **33 bước** | **~14-18 ngày** |
