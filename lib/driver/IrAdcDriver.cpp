#include "IrAdcDriver.h"
#include <Wire.h>

bool IrAdcDriver::probe() {
    Wire.beginTransmission(ADDRESS);
    return Wire.endTransmission() == 0;
}

bool IrAdcDriver::startSingleEnded(uint8_t channel) {
    if (channel > 2) return false;

    // OS=1, MUX=AIN0/1/2-to-GND, PGA=+/-4.096V, single-shot,
    // 860 SPS, comparator disabled. Conversion must finish before read.
    const uint16_t config = 0x83E3u | (uint16_t)(4u + channel) << 12;
    Wire.beginTransmission(ADDRESS);
    Wire.write((uint8_t)0x01); // Configuration register
    Wire.write((uint8_t)(config >> 8));
    Wire.write((uint8_t)(config & 0xFF));
    return Wire.endTransmission() == 0;
}

bool IrAdcDriver::readConversion(uint16_t& raw) {
    Wire.beginTransmission(ADDRESS);
    Wire.write((uint8_t)0x00); // Conversion register
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(ADDRESS, (uint8_t)2) != 2) return false;
    raw = (uint16_t)((uint16_t)Wire.read() << 8) | (uint16_t)Wire.read();
    return true;
}
