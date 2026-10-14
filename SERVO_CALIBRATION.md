# Servo vòi GPIO38 (ESP32-S3 N8R2)

File chỉnh tham số: `lib/common/ServoConfig.h`. Firmware thường hiện có
`SERVO_SWEEP_ENABLED 1`: **PUMP ON hợp lệ sẽ điều khiển cả GPIO38**. Firmware thử
góc dùng env riêng `freenove_esp32_s3_wroom_n8r2_servo_cal`; nó không chạy
motor, bơm, bridge USB CDC, IMU hay các task điều khiển robot.
PWM servo sử dụng LEDC kênh 6, 50 Hz, 14 bit theo giới hạn ESP32-S3;
không đổi sang 16 bit vì `ledcSetup()` sẽ thất bại.

## 1. Thử góc theo từng bước

1. Dừng launch của Pi và các lệnh điều khiển robot, ngắt nguồn riêng của bơm.
   Khi nạp firmware thử góc, Pi sẽ tạm mất `/odom`, `/imu/data` và `/fire_sensors/ir_raw`.
2. Trong Antigravity/PlatformIO, chọn environment
   `freenove_esp32_s3_wroom_n8r2_servo_cal` rồi Upload qua cổng đã dùng để nạp
   S3 trước đây. **Không** chọn environment thường cho bước này.
3. Mở Serial Monitor trên cổng debug CH340/Serial0, baud `115200`, line ending
   `Newline` hoặc `Both NL & CR`. Xác nhận hiện `[SERVO-CAL]` và servo chưa
   chuyển động sau khởi động.
4. Gửi `help`, rồi bắt đầu ở một góc bạn vừa quan sát là an toàn (thường
   `a 90`). Firmware nhận trực tiếp mọi góc nguyên trong dải 40–140 độ,
   không ép chênh lệch 5 độ giữa hai lệnh. Khi thử thực tế, vẫn nên tự tăng
   giảm từng vài độ để quan sát trước khi đi xa hơn.
   Gửi `off` để ngắt xung. Mỗi lệnh góc giữ tối đa 10 giây, sau đó tự ngắt
   xung; không tự quét. Sau khi khởi động lại, lệnh góc đầu tiên cần chính
   bạn chọn gần vị trí vòi đang ở; firmware không biết góc vật lý thực.
5. Quan sát chính **hướng tia vòi**, ghi góc thấp và góc cao *không chạm cữ
   cơ khí*. Dải thử thủ công hiện cho phép là 40–140 độ. Dừng ngay nếu servo kêu gằn, tay đòn
   kẹt hoặc vòi vướng khung. Hai giá trị 40/140 chỉ là giới hạn phần mềm được
   mở theo yêu cầu, **không phải góc an toàn đã được xác nhận**.

Ví dụ bản ghi kết quả:

```text
Nozzle low:  ___ deg
Nozzle high: ___ deg
Safe test bounds: ___ .. ___ deg
```

## 2. Sau khi xác nhận góc

Góc đã đo: vòi thấp `115°`, vòi cao `45°`. Hành trình đã chốt là
`90° → 45° → 115°` trong một lần bơm. Thuật toán chỉ đổi tối đa **1° mỗi
lần**, tối thiểu 35 ms giữa các bước. Với task Serial 20 ms đều, thực tế
khoảng 40 ms/bước: tới 45° sau 1,8 giây, tới 115° sau 4,6 giây. Task bị
trễ thì không nhảy bù nhiều độ; có thể chưa tới 115° trước khi bơm tắt.

**Trước lần nạp/chạy bản thường đầu tiên, đặt vòi thực tế gần 90°
bằng firmware hiệu chuẩn.** ESP32-S3 không có phản hồi vị trí servo: lệnh
đầu tiên `90°` vẫn có thể giật nếu vòi thực tế đang ở xa góc đó. Không
thử khi tay đòn hoặc vòi bị cấn cơ khí. Giới hạn 1° chỉ áp dụng sau lệnh
đầu tiên, không bảo đảm servo không trượt răng nếu cơ cấu chịu lực lớn.

Nạp lại environment **thường** `freenove_esp32_s3_wroom_n8r2`. Không để
firmware hiệu chuẩn chạy khi cần Pi/ROS. Bản thường quét một lượt, chỉ khi
lệnh bơm qua các khóa an toàn hiện hành được
cho phép và PWM bơm đã khởi tạo. Sau khi quét **đủ tới 115°**, bơm tắt do
PUMP OFF hoặc giới hạn 5 giây, vòi đi từ 115° về 90° từng 1° (ít nhất
35 ms/bước), giữ 90° thêm 300 ms rồi ngắt PWM. Cần có PUMP OFF trước
một lệnh ON mới; ON giữ liên tục không tạo thêm lượt phun. Trong lúc hồi,
bơm luôn OFF; máy chủ/watchdog còn hoạt động và lệnh chuyển động xe phải
bằng zero. Nếu phun dừng trước 115°, mất kết nối, quá hạn lease, xe nhận
lệnh di chuyển hoặc lỗi PWM servo thì dừng PWM và khóa lượt tới khi reboot;
đặt vòi thực tế gần 90° trước khi thử lại. Hồi về là lệnh góc mở vòng,
không phải bằng chứng vòi đã thật sự về 90°. Firmware không đo
phản hồi dòng nước hay vị trí thực của servo. Giới hạn bơm 5 giây trong
`PumpSafety` không thay đổi.

ROS hiện vẫn có các node `one_shot` cho Nav2 stop, cancel, alignment và
pump demo. Hồi vòi trên S3 **không tự reset** những node đó để nhận goal/lửa
thứ hai; lượt thứ hai cần một phiên ROS mới được khởi động an toàn sau khi
bơm OFF, xe đứng yên và không còn goal cũ hoạt động.

## 3. Kiểm tra trước khi nạp

```bash
cd /home/huudinh/Documents/esp_projects/robot_mainboard
/home/huudinh/.platformio/penv/bin/platformio run -e freenove_esp32_s3_wroom_n8r2_servo_cal
/home/huudinh/.platformio/penv/bin/platformio run -e freenove_esp32_s3_wroom_n8r2
/home/huudinh/.platformio/penv/bin/platformio test -e native -f test_servo_sweep
```

Các lệnh này chỉ build/test trên laptop, không nạp và không kích phần cứng.
