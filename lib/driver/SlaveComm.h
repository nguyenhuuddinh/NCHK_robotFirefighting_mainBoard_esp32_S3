#pragma once

#include "DataStructs.h"
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Giao tiep UART giua ESP32-S3 va ESP32-WROOM.
 *
 * Chuc nang:
 *   - Nhan SensorPacket_t tu WROOM (header 0xAA + CRC8)
 *   - Gui ActuatorCmd_t xuong WROOM (header 0xBB + CRC8)
 *
 * Lop nay thuoc tang driver/ — chi giao tiep phan cung,
 * khong chua logic nghiep vu (theo quy tac Layer).
 */
class SlaveComm {
public:
    /**
     * @brief Khoi tao UART2 tren GPIO 19 (TX) / 20 (RX).
     * Baudrate 115200, 8N1.
     */
    void init();

    /**
     * @brief Thu doc 1 SensorPacket_t tu buffer UART.
     * Tim header 0xAA, doc 14 bytes, verify CRC8.
     * @param out  Packet hop le (chi co gia tri khi return true)
     * @return true neu parse + CRC thanh cong
     */
    bool tryReceive(SensorPacket_t& out);

    /**
     * @brief Gui lenh dieu khien xuong WROOM.
     * Tu dong dong goi header 0xBB + CRC8.
     * @param servoPan  Goc xoay voi phun (0-180, 90=center)
     * @param pumpOn    Bat/tat bom (true=bat)
     * @param buzzerOn  Bat/tat buzzer (true=bat)
     */
    void sendCommand(uint8_t servoPan, bool pumpOn, bool buzzerOn);

    /**
     * @brief Lay thong ke UART de debug.
     */
    uint32_t getPacketsOk() const   { return _packetsOk; }
    uint32_t getPacketsFail() const { return _packetsFail; }

    /**
     * @brief Tinh CRC8 (Polynomial 0x07).
     * Ham static de co the dung chung cho ca send va receive.
     */
    static uint8_t calcCRC8(const uint8_t* data, size_t len);

private:
    uint32_t _packetsOk   = 0;  // So packet nhan thanh cong
    uint32_t _packetsFail = 0;  // So packet CRC fail
};

// Instance toan cuc (tuong tu encoderDriver, motorDriver)
extern SlaveComm slaveComm;
