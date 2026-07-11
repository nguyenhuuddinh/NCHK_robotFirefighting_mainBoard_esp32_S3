#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <Arduino.h>
#include <ESP32Encoder.h>
#include "../common/PinConfig.h"

class EncoderDriver {
public:
    EncoderDriver();
    void init();
    
    // Trả về số xung đếm được (Sử dụng kiểu int64_t 64-bit để vĩnh viễn không bao giờ bị tràn số)
    int64_t getCountFL();
    int64_t getCountRL();
    int64_t getCountFR();
    int64_t getCountRR();
    
    // Reset bộ đếm về 0
    void resetAll();

private:
    ESP32Encoder encFL;
    ESP32Encoder encRL;
    ESP32Encoder encFR;
    ESP32Encoder encRR;
};

extern EncoderDriver encoderDriver;

#endif
