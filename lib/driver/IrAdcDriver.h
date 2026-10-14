#ifndef IR_ADC_DRIVER_H
#define IR_ADC_DRIVER_H

#include <stdint.h>

// ADS1115 at ADDR=GND. The IMU initializes the shared Wire bus first.
class IrAdcDriver {
public:
    bool probe();
    bool startSingleEnded(uint8_t channel);
    bool readConversion(uint16_t& raw);

private:
    static constexpr uint8_t ADDRESS = 0x48;
};

#endif
