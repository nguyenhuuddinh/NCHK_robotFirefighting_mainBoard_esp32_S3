#include "SlaveComm.h"
#include "PinConfig.h"
#include "RobotConfig.h"
#include <Arduino.h>
#include <HardwareSerial.h>

// Instance toan cuc
SlaveComm slaveComm;

// Su dung UART2 cua ESP32-S3
static HardwareSerial& uartWroom = Serial2;

// ============================================================
// INIT
// ============================================================
void SlaveComm::init() {
    uartWroom.begin(115200, SERIAL_8N1, SLAVE_RX_PIN, SLAVE_TX_PIN);

    // Cho UART on dinh
    vTaskDelay(pdMS_TO_TICKS(100));

    _packetsOk = 0;
    _packetsFail = 0;

    DBG.printf("[OK] SlaveComm UART2 initialized (TX=%d, RX=%d, 115200)\n",
                  SLAVE_TX_PIN, SLAVE_RX_PIN);
}

// ============================================================
// CRC8 (Polynomial 0x07 — CRC-8 chuan)
// ============================================================
uint8_t SlaveComm::calcCRC8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// ============================================================
// RECEIVE: Doc SensorPacket_t tu WROOM
// ============================================================
bool SlaveComm::tryReceive(SensorPacket_t& out) {
    // Can it nhat sizeof(SensorPacket_t) = 14 bytes trong buffer
    if (uartWroom.available() < (int)sizeof(SensorPacket_t)) {
        return false;
    }

    // Tim header 0xAA
    while (uartWroom.available() >= (int)sizeof(SensorPacket_t)) {
        uint8_t peek = uartWroom.peek();
        if (peek == SENSOR_PACKET_HEADER) {
            break; // Tim thay header
        }
        uartWroom.read(); // Bo byte rac, tim tiep
    }

    // Kiem tra con du byte khong
    if (uartWroom.available() < (int)sizeof(SensorPacket_t)) {
        return false;
    }

    // Doc toan bo packet
    uint8_t buf[sizeof(SensorPacket_t)];
    size_t bytesRead = uartWroom.readBytes(buf, sizeof(SensorPacket_t));

    if (bytesRead != sizeof(SensorPacket_t)) {
        _packetsFail++;
        return false;
    }

    // Verify CRC8 (tinh tren tat ca byte tru byte cuoi cung la CRC)
    uint8_t calcCrc = calcCRC8(buf, sizeof(SensorPacket_t) - 1);
    uint8_t recvCrc = buf[sizeof(SensorPacket_t) - 1];

    if (calcCrc != recvCrc) {
        _packetsFail++;
        return false;
    }

    // Copy vao output struct
    memcpy(&out, buf, sizeof(SensorPacket_t));
    _packetsOk++;
    return true;
}

// ============================================================
// SEND: Gui ActuatorCmd_t xuong WROOM
// ============================================================
void SlaveComm::sendCommand(uint8_t servoPan, bool pumpOn, bool buzzerOn) {
    ActuatorCmd_t cmd;
    cmd.header    = ACTUATOR_CMD_HEADER;
    cmd.servo_pan = servoPan;
    cmd.pump_on   = pumpOn ? 1 : 0;
    cmd.buzzer_on = buzzerOn ? 1 : 0;

    // Tinh CRC tren 4 byte dau (header + servo + pump + buzzer)
    cmd.crc8 = calcCRC8((const uint8_t*)&cmd, sizeof(ActuatorCmd_t) - 1);

    uartWroom.write((const uint8_t*)&cmd, sizeof(ActuatorCmd_t));
}
