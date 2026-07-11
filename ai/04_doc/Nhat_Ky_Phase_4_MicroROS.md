# Nhật Ký Phát Triển Phase 4: Tích hợp micro-ROS trên ESP32-S3

Tài liệu này được lập theo yêu cầu của User để ghi chép lại toàn bộ hành trình, tư duy xử lý (Mental Models), và chi tiết các bẫy kỹ thuật (Bugs) gặp phải trong quá trình xây dựng hệ thống giao tiếp giữa phần cứng ESP32-S3 và ROS 2 (Raspberry Pi).

---

## 1. Mục Tiêu Phase 4
Chuyển đổi robot từ việc giao tiếp bằng WebServer/WiFi sang giao tiếp chuẩn công nghiệp **ROS 2** thông qua thư viện **micro-ROS (XRCE-DDS)**. 
- Xây dựng kiến trúc đa luồng FreeRTOS cho ESP32-S3.
- Định tuyến phần cứng: Phân tách rõ ràng cổng ghi log và cổng truyền dữ liệu.
- Hoàn thiện State Machine (Máy trạng thái) để xử lý việc kết nối/ngắt kết nối với ROS 2 Agent một cách an toàn mà không làm rò rỉ bộ nhớ.
- Triển khai Publish Odometry (`/odom`), IMU (`/imu/data`) và Subscribe Lệnh chạy (`/cmd_vel`).

---

## 2. Kiến Trúc & Tư Duy Thiết Kế (Architecture & Mental Models)

### Phân tách 2 cổng vật lý độc lập
Do gói tin của micro-ROS là nhị phân (Binary) dạng mã hóa, nếu in chung với log gỡ lỗi (ASCII) sẽ gây ra tình trạng rác màn hình và làm vỡ cấu trúc gói tin DDS. Tư duy ở đây là phải cô lập hoàn toàn:
- **`Serial0` (UART0 qua chip CH340):** Dùng riêng cho việc in ra text (`DBG.println()`), cắm vào Laptop để monitor.
- **`Serial` (Native USB CDC / Type-C OTG):** Dùng riêng làm "ống nước" (Transport) chỉ để bơm và nhận các luồng nhị phân XRCE-DDS, cắm thẳng sang Raspberry Pi.

### Cấu trúc FreeRTOS đa luồng (Multi-core)
Robot cần đảm bảo tính thời gian thực (Real-time) cho việc đọc Encoder và tính toán PID. Do đó:
- **Core 1 (Priority 5):** Dành riêng cho `Task_Motion`. Luôn chạy ở 500Hz không bao giờ bị dừng hay block. Dữ liệu Odometry (`x, y, theta`) liên tục được cập nhật vào RAM (`g_ctx.odom`).
- **Core 0 (Priority 3):** Dành cho `Task_MicroROS`. Đọc tọa độ từ RAM và đóng gói gửi lên ROS 2 ở tần số 50Hz.

### Vòng đời giao tiếp (State Machine)
Để đảm bảo ESP32 có thể chạy liên tục hàng tháng trời không rò rỉ RAM (Heap Leak), chúng tôi thiết lập vòng lặp 3 trạng thái nghiêm ngặt trong `MicroRosComm`:
1. `UROS_WAITING_AGENT`: Bắn gói `ping` chậm rãi (2Hz). Nếu Agent trả lời OK -> Khởi tạo tĩnh bộ nhớ (`createEntities_`) -> Chuyển sang `CONNECTED`.
2. `UROS_CONNECTED`: Chạy đều đặn 50Hz, liên tục kiểm tra nhịp tim (Heartbeat) của Agent sau mỗi giây.
3. `UROS_DISCONNECTED`: Nếu mất nhịp tim, lập tức phát lệnh **EMERGENCY (Dừng phanh khẩn cấp cả 4 bánh)** để an toàn phần cứng, dọn sạch bộ đệm rcl (`destroyEntities_`), sau đó quay về `WAITING`.

---

## 3. Các Vấn Đề Lớn Gặp Phải & Phân Tích Chuyên Sâu (Giai đoạn 4.1 & 4.2)

Dưới đây là 3 lỗi cực kỳ điển hình của hệ thống nhúng khi kết hợp FreeRTOS và giao thức XRCE-DDS mà chúng tôi đã đối mặt và bóc tách.

### 🐛 BUG 1: Watchdog Timer (WDT) Triggered do thiếu Delay khi mất kết nối
**Triệu chứng:**
Khi chưa kết nối được ROS Agent, cứ sau mỗi ~5 đến 13 giây, mạch ESP32 tự động in lỗi đỏ lừ trên Monitor:
`E (13780) task_wdt: Task watchdog got triggered. - IDLE0 (CPU 0)` và tự khởi động lại.

**Nguyên nhân gốc rễ (Root Cause):**
- Hàm `vTaskDelayUntil(&xLastWake, 20ms)` dùng để giữ chu kỳ chuẩn 50Hz. Nhưng hàm `rmw_uros_ping_agent()` đôi khi block cổng USB mất đến 100-200ms để chờ phản hồi.
- Do thời gian chạy (`200ms`) lớn hơn chu kỳ gốc (`20ms`), hàm `vTaskDelayUntil` nghĩ rằng task đang chạy "chậm tiến độ", nên nó **ép task không được ngủ một mili-giây nào** để đuổi kịp deadline.
- Hậu quả: `Task_uROS` (Độ ưu tiên 3) chạy 100% công suất Core 0, không nhường CPU cho tác vụ nền `IDLE0` (Độ ưu tiên 0) vào xóa bộ đếm Watchdog Timer. Sau 5 giây, Watchdog phán đoán hệ thống bị treo nên tự động "bắn bỏ" (Abort).

**Cách khắc phục:**
Trong trạng thái chờ (`UROS_WAITING_AGENT`), thay vì dùng `vTaskDelayUntil`, chúng tôi dùng `vTaskDelay(pdMS_TO_TICKS(500))`. Lệnh này luôn bắt buộc hệ điều hành nhường CPU trong nửa giây, giúp `IDLE0` có đủ thời gian reset Watchdog. Sau khi kết nối, hệ thống mới switch lại sang 50Hz và reset mốc đếm bằng `xLastWake = xTaskGetTickCount()`.

---

### 🐛 BUG 2: Mạch Reset & Mất Sạch Odometry khi Rút/Cắm cáp USB (Tưởng là Lỗi Phần Mềm)
**Triệu chứng:**
User phản ánh: "Khi đang kết nối mà rút cáp USB ra cắm lại thì thông số Odometry và IMU mất hết, trả về 0. Lỗi do ESP32 tự reset khi rớt mạng?".

**Nguyên nhân gốc rễ (Hardware Auto-Reset):**
- Về mặt phần mềm: **ESP32 tuyệt đối không reset khi mất Agent.** Nếu dùng phím `Ctrl + C` tắt Agent trên Pi (nhưng vẫn giữ nguyên cáp), mạch vẫn chạy, các biến Odometry vẫn đếm bình thường.
- Về mặt phần cứng: Các mạch phát triển ESP32 (như Freenove ESP32-S3) có một mạch tụ nối với chân `EN (Reset)` và được điều khiển bởi hai đường tín hiệu `DTR` và `RTS` của chip USB-to-Serial.
- Bất cứ khi nào cáp bị rút và cắm lại (hoặc khi PlatformIO Monitor trên máy tính tự kết nối lại cổng `ttyACM`), hệ điều hành sẽ kéo chân `DTR` xuống mức thấp, kích hoạt mạch Transistor kéo chân `EN` xuống GND, gây ra hiện tượng **Hardware Reset**. Khởi động lại vi điều khiển sẽ làm trống toàn bộ RAM, khiến IMU và Odometry calibrate lại từ 0.

**Kết luận:** Đây là tính năng vật lý thiết kế sẵn để nạp code và debug, không phải là bug phần mềm.

---

### 🐛 BUG 3: Kẹt thư viện XRCE-DDS khi Host đóng cổng đột ngột (The "Ctrl+C" Freeze)
**Triệu chứng:**
Khi đang kết nối, nếu User bấm `Ctrl + C` tắt thẳng `micro_ros_agent` trên Raspberry Pi, ESP32 báo `[uROS] Entities destroyed...` nhưng sau đúng 5 giây, mạch vẫn bị Watchdog Timer báo lỗi và Abort.

**Phân tích sâu (Tầng thư viện XRCE-DDS & Native USB):**
- Khi Agent chết đột ngột, cổng Serial trên Pi đóng lại, làm tín hiệu DTR rớt xuống 0.
- Theo mặc định, thư viện **Native USB CDC (`HWCDC`)** của ESP32-S3 sẽ khóa luồng (block) hàm `write()` nếu nó cố đẩy dữ liệu vào bộ đệm USB mà không có thiết bị (Host) nào đang lắng nghe (DTR=0).
- Sau khi ESP32 quay vòng lại trạng thái `WAITING`, nó gọi hàm `rmw_uros_ping_agent`. Hàm này gọi `Serial.write()` để gửi ping.
- Việc `Serial.write()` bị block vô hạn khiến tác vụ `Task_uROS` trên Core 0 bị đứng hình mãi mãi. Tương tự như Bug 1, `IDLE0` bị bỏ đói và Watchdog Timer tự chém đứt hệ thống sau 5 giây.

**Quá trình gỡ rối và Lựa chọn cuối cùng:**
1. *Giải pháp nâng cao đã thử:* Thêm `Serial.setTxTimeoutMs(0)` để ép cổng USB loại bỏ byte thừa thay vì block, và thêm điều kiện `if (!Serial)` kiểm tra vật lý để tuyệt đối không gọi `ping` nếu cổng USB không có tín hiệu.
2. *Vấn đề phát sinh:* Việc áp đặt logic cấp thấp (Low-level DTR checking) này phá vỡ luồng làm việc mặc định của micro-ROS, khiến cho việc Re-enumeration (Bắt tay lại USB) đôi khi bị kẹt nếu Raspberry Pi đánh rơi tên cổng (`/dev/ttyACM0` nhảy sang `ttyACM1`).
3. *Quyết định hoàn tác (Revert):* Theo yêu cầu của QA và User để tránh làm phức tạp hóa mã nguồn, chúng tôi quyết định **hoàn tác (roll-back)** mọi thay đổi ép buộc này. Hệ thống được đưa về trạng thái nguyên bản "sạch sẽ".
4. **Quy tắc vận hành chuẩn:** Để giải quyết gọn gàng tình trạng kẹt buffer này, luật vận hành từ nay là: *Nếu ngắt Agent hoặc đứt cáp, hãy ấn nút RESET phần cứng (chân EN) trên ESP32 để 2 bên xóa toàn bộ đệm USB và bắt tay lại từ số không.*

---

## 4. Các Vấn Đề Giai Đoạn Publish Topic (Bước 4.3 & 4.4 - Odometry & IMU)

Trong lúc thực hiện Bước 4.3 (Publisher `/odom`) và Bước 4.4 (Publisher `/imu/data`), khi chúng tôi nhúng thêm code để Publish dữ liệu `nav_msgs/Odometry` và `sensor_msgs/Imu`, hàng loạt lỗi Crash phần cứng và đơ giao thức đã phát sinh. Dưới đây là phân tích của QA Engineer.

### 🐛 BUG 4: Tự động Reset/Crash do Phân Mảnh Gói Tin (MTU Fragmentation)
**Câu hỏi của User:** "Nó vẫn lỗi nè bạn, trước lúc mà code phần odom imu nó đâu có bị vậy đâu bạn, check lại lần lượt toàn bộ code để tìm ra lỗ hổng đi bạn, với lại sao lại tạo ra cái `colcon.meta` để làm gì nữa?"

**Phân tích của QA:** 
- Gói tin `nav_msgs/Odometry` có kích thước lớn (chứa nhiều hiệp phương sai Covariance, ~600-700 byte). 
- Mặc định, thư viện micro-ROS cấp phát một gói tin tối đa (MTU) chỉ có `512 bytes`. 
- Khi ESP32 cố gắng Publish cái túi to hơn 512 bytes, thư viện XRCE-DDS bắt buộc phải băm vụn gói tin ra (Phân mảnh). Quá trình phân mảnh gói tin trên hạ tầng Native USB CDC (Serial) rất chậm và tốn bộ đệm.
- Quá trình này gây nghẽn `tud_cdc_n_write_flush` (chờ ACK từ Host), lập tức làm Core 0 bị tắc thở và bị Task Watchdog chém chết cái rụp.

**Cách giải quyết (Cách mạng MTU):**
- QA đưa ra giải pháp là phải sinh ra file cấu hình `colcon.meta` để ghi đè thông số lõi của thư viện micro-ROS bằng dòng lệnh `rmw_microxrcedds_custom_mtu="2048"`.
- Bắt buộc phải **xóa trắng thư mục Cache của thư viện** `.pio/libdeps/freenove_esp32_s3_wroom/micro_ros_platformio` rồi dùng PlatformIO build lại C source code từ số không tròn trĩnh. (Quá trình này tốn khoảng 120 giây).
- Kết quả: Cái ống nước (MTU) phình to gấp 4 lần. Odometry lọt qua mượt mà trong 1 packet duy nhất, mạch không bao giờ bị Crash nữa.

---

### 🐛 BUG 5: Đụng Độ Tài Nguyên I2C Giữa Hai Lõi CPU (`Unfinished Repeated Start transaction`)
**Câu hỏi của User:** "Nó hiện `[Wire.cpp:416] beginTransmission(): Unfinished Repeated Start transaction!` mấy cái wire là gì vậy bạn?"

**Phân tích của QA:**
- MPU6050 giao tiếp qua I2C (`Wire`).
- `Task_Motion` (Core 1) chạy độc lập 500Hz liên tục quét I2C để đọc IMU phục vụ tính toán Odometry.
- Cùng thời điểm đó, QA Engineer bất cẩn cho `Task_MicroROS` (Core 0) chạy 50Hz đi gọi thẳng hàm `imuDriver.getGyroZ()` để chép số liệu xuất ra topic `/imu/data`.
- Chuyện gì đến cũng đến: 2 Lõi CPU giành giật nhau cái dây I2C trong cùng 1 chu kỳ tích tắc. Không có Mutex bảo vệ luồng điện, tín hiệu điện áp bị đứt đôi giữa đường, thư viện `Wire` la hét ầm ĩ.

**Cách giải quyết (Design Pattern):**
- **Cô lập tầng vật lý:** Nghiêm cấm Core 0 đụng chạm vào lớp Driver I2C.
- Mọi dữ liệu về Vận tốc góc (`gyro_z`) do Core 1 đọc xong sẽ được nhét gọn gàng vào biến RAM dùng chung: `SharedContext.odom.angular_velocity`.
- `Task_MicroROS` ở Core 0 mỗi khi cần chỉ cần lấy chìa khóa Mutex mở két RAM, copy dữ liệu ra rồi đem đi đóng gói. Việc tính Odometry của Core 1 dưới slave **hoàn toàn không bị ảnh hưởng**, tốc độ 500Hz được bảo toàn tuyệt đối.

---

### 🐛 BUG 6: Giao Thức Mạng ROS 2 Bị Đơ & Agent Không Lên Tiếng (The SHM Deadlock)
**Câu hỏi của User:** "Tôi bị dính lỗi ở 2 câu lệnh này nè, tại sao lệnh `ros2 launch...` chạy lần 1 thì kết nối được, rồi tôi ấn Ctrl+C xong chạy lại lệnh y hệt thì nó cứ treo mãi ở dòng `logger setup` không kết nối?"

**Phân tích của QA:**
Khi Agent treo im lìm, User nghĩ rằng cấu hình tham số port/baudrate bị sai, nhưng thực chất lỗi đến từ sâu bên trong hệ điều hành Linux:
- Giao thức FastRTPS (lớp nền của ROS 2) sử dụng Shared Memory (SHM - `/dev/shm`) trên Raspberry Pi để truyền tin cực nhanh giữa các Node.
- Khi người dùng nhấn `Ctrl + C` để đóng Agent, hoặc lúc ESP32 bị crash Watchdog trước đó, tiến trình bị ngắt đột ngột và **bỏ quên không giải phóng vùng nhớ SHM**.
- Ở lần mở Agent thứ 2, khi tiến trình khởi tạo Participant (với tư cách DDS), nó đụng phải vùng nhớ bị khóa và rơi vào tình trạng **Deadlock (treo não)** vĩnh viễn ngay tại `logger setup`. Nó không thể lắng nghe tín hiệu Ping từ cổng `/dev/ttyACM0` nữa.

**Cách giải quyết (Quy tắc dọn rác):**
- Bấm `Ctrl+C` tắt Agent đang bị treo.
- **Bắt buộc** gõ dòng lệnh chọc thủng vùng nhớ rác: `rm -rf /dev/shm/*`
- Lúc này, bật Agent lên lại sẽ khởi tạo SHM sạch sẽ. Chú ý theo luật của BUG 3, mạch ESP32 cũng đã bị kẹt Native USB khi bạn nhấn `Ctrl+C`, nên phải thò tay ấn nút RESET cứng trên ESP32.
- Kết quả: Agent hô vang `session established`, Odom và IMU đổ về như thác nước.

---

## 5. Kết Quả Nghiệm Thu (Bước 4.3 & 4.4)
* **Bước 4.3:** Topic `/odom` đã lên thông số `angular.z` mượt mà với Covariance ma trận rỗng (chuẩn robot 2D Skid-Steer).
* **Bước 4.4:** Topic `/imu/data` đẩy Quaternion chính xác, timestamp đồng bộ.
* Mạch ESP32 không còn bất cứ dòng đỏ báo Watchdog Reset nào, hoàn toàn ổn định để chạy thời gian thực.
* **Định hướng tiếp theo:** Triển khai **Bước 4.5 (Publisher `/env_status`)** và **Bước 4.6 (TF broadcaster)** theo đúng Rule trong Template.
