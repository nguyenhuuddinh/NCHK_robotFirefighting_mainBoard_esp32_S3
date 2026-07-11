#include "EncoderDriver.h"

EncoderDriver encoderDriver;

EncoderDriver::EncoderDriver() {}

void EncoderDriver::init() {
    // Bắt buộc trên ESP32-S3 khi dùng thư viện ESP32Encoder để tránh nhiễu
    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    // Attach các kênh A B theo chế độ Full Quadrature (Đếm cả 4 sườn xung)
    encFL.attachFullQuad(ENC1_A_PIN, ENC1_B_PIN);
    encRL.attachFullQuad(ENC2_A_PIN, ENC2_B_PIN);
    encFR.attachFullQuad(ENC3_A_PIN, ENC3_B_PIN);
    encRR.attachFullQuad(ENC4_A_PIN, ENC4_B_PIN);
    
    // Kích hoạt bộ lọc nhiễu phần cứng (Glitch Filter) của bộ đếm PCNT ESP32.
    // Bỏ qua các xung nhiễu ngắn hơn 1023 chu kỳ xung nhịp APB (khoảng 12.7 micro-giây)
    // Cực kỳ hữu ích để chống nhiễu từ tia lửa điện chổi than của động cơ L298N.
    encFL.setFilter(1023);
    encRL.setFilter(1023);
    encFR.setFilter(1023);
    encRR.setFilter(1023);
    
    resetAll();
}

int64_t EncoderDriver::getCountFL() {
    return encFL.getCount();
}

int64_t EncoderDriver::getCountRL() {
    return encRL.getCount();
}

int64_t EncoderDriver::getCountFR() {
    return encFR.getCount();
}

int64_t EncoderDriver::getCountRR() {
    return encRR.getCount();
}

void EncoderDriver::resetAll() {
    encFL.clearCount();
    encRL.clearCount();
    encFR.clearCount();
    encRR.clearCount();
}
