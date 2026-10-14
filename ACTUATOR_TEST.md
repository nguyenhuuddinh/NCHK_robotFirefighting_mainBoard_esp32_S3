# Thử riêng bơm GPIO47 và servo GPIO38 trên ESP32-S3 N8R2

Environment `freenove_esp32_s3_wroom_n8r2_actuator_test` chỉ chạy bài thử
cơ cấu. Nó **không** chạy bridge USB CDC, odom, IMU, motor hoặc ROS. Đừng dùng
environment này cho demo Nav2. Firmware thường vẫn là
`freenove_esp32_s3_wroom_n8r2`.

## Chuẩn bị

1. Dừng launch Pi và mọi lệnh điều khiển robot; xác nhận xe đứng yên và ngắt
   nguồn driver bánh. Pi sẽ tạm mất `/odom`/`imu/data` khi nạp firmware thử.
2. **Ngắt nguồn riêng của bơm** cho lượt thử đầu. Giữ nguồn servo, nối GND
   chung với S3. Đặt vòi thực tế gần 90 độ trước khi bật thử vì servo không có
   phản hồi vị trí; lệnh góc đầu có thể giật nếu vòi đang xa 90 độ.
3. Chọn environment `freenove_esp32_s3_wroom_n8r2_actuator_test` trong
   Antigravity/PlatformIO và Upload bằng cổng S3 native USB đã dùng trước đó.
   Hoặc build (không nạp) bằng:

   ```bash
   cd /home/huudinh/Documents/esp_projects/robot_mainboard
   /home/huudinh/.platformio/penv/bin/platformio run -e freenove_esp32_s3_wroom_n8r2_actuator_test
   ```

4. Mở Serial Monitor của **cáp debug CH340/Serial0**, baud 115200, line ending
   Newline. Không gửi lệnh ROS `/pump_cmd` cho environment này. Sau boot, cả
   GPIO47 và GPIO38 OFF; xe không tự chạy, bơm không tự bật.

## Một lượt thử khô

Gửi từng dòng riêng trong Serial Monitor:

```text
help
status
off
run
```

`off` đầu tiên là điều kiện cấp lượt. Sau `run`, GPIO47 lên mức ON và servo
đi `90 -> 45 -> 115` từng tối đa 1 độ/bước, cách nhau ít nhất 35 ms. Ở mốc
5 giây, GPIO47 tự OFF; servo hồi `115 -> 90`, giữ 300 ms rồi ngắt PWM. Chờ
`GPIO38 at commanded 90; PWM OFF`, sau đó gửi:

```text
off
status
run
```

Lượt thứ hai phải chạy lại mà không cần reboot. `off` khi đang chạy/quay về
là dừng khẩn cả hai output và khóa phiên cho tới khi reboot; không gửi `off`
trước khi vòi đã hồi xong nếu mục tiêu là thử tái cấp lượt. Nếu thấy servo
kẹt, giật mạnh, cấn cơ khí hoặc bơm bất thường thì gửi `off` ngay.

Khi ngắt nguồn bơm, **bơm không thể chạy vật lý**. Có thể dùng đồng hồ DC đo
GPIO47 so với GND S3: gần 3,3 V lúc `run`, về 0 V sau 5 giây. Trên GPIO38
là xung 50 Hz; đồng hồ DC không xác nhận được góc, hãy quan sát vòi.

Chỉ sau khi lượt khô đạt mới cân nhắc thử có nước, tại khu vực an toàn,
người vận hành ở cạnh nút ngắt nguồn bơm. Firmware thử này không phát hiện
lửa, không kiểm tra IR, không tự dập lửa.

## Khôi phục ROS/Pi

Nạp lại **environment thường** `freenove_esp32_s3_wroom_n8r2`, rồi bật lại
launch Pi. Xác nhận `/odom` và `/imu/data` khoảng 10 Hz trước khi dùng Nav2.
Không để firmware thử cơ cấu trên S3 khi vận hành robot.
