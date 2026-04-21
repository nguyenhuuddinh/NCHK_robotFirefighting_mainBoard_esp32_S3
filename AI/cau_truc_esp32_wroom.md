Firefighting_Sensor_WROOM/
├── lib/
│   ├── app/
│   │   ├── TaskManager.h / .cpp    # Quản lý Task_Sensor, Task_Radio, Task_Comms.
│   │   └── SafetyMonitor.h / .cpp  # Đánh giá mức độ nguy hiểm (Lửa sau lưng, Gas cao) để kích hoạt Buzzer.
│   │
│   ├── common/
│   │   ├── DataStructs.h           # Struct chứa dữ liệu môi trường và Lệnh RF.
│   │   └── PinConfig.h             # Mapping chân: SPI NRF24, ADC Gas/Pin, Digital IR, Buzzer.
│   │
│   ├── driver/
│   │   ├── RadioDriver.h / .cpp    # Giao tiếp SPI với NRF24L01+.
│   │   ├── SensorDriver.h / .cpp   # Đọc các cảm biến (MQ Gas, Nhiệt độ, Flame IR, Điện áp Pin).
│   │   ├── BuzzerDriver.h / .cpp   # Điều khiển còi chip.
│   │   └── MasterComm.h / .cpp     # UART DMA gửi dữ liệu đóng gói sang cho ESP32-S3.
│   │
│   └── service/
│       ├── DataFilter.h / .cpp     # Lọc nhiễu ADC (EMA) và chống rung (Debounce) cho cảm biến Lửa IR.
│       └── ProtocolPacker.h / .cpp # Gói gọn Cảm biến + Lệnh RF vào 1 Struct, đóng Checksum gửi UART.
│
├── src/
│   └── main.cpp
└── platformio.ini                  # Khai báo lib RF24.