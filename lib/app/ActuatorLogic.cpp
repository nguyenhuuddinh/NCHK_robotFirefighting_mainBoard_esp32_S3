#include "ActuatorLogic.h"
#include "DataStructs.h"
#include <Arduino.h>

uint8_t ActuatorLogic::fireXToServoPan(float fireX) {
    // Clamp ve [-1, 1]
    if (fireX > 1.0f) fireX = 1.0f;
    if (fireX < -1.0f) fireX = -1.0f;

    // Map: -1.0 -> 0°, 0.0 -> 90°, 1.0 -> 180°
    float angle = (fireX + 1.0f) * 90.0f;

    // Clamp ve [0, 180] an toan
    if (angle > 180.0f) angle = 180.0f;
    if (angle < 0.0f) angle = 0.0f;

    return (uint8_t)angle;
}

void ActuatorLogic::processFireTarget(float fireX, bool pumpOn, bool buzzerOn,
                                      QueueHandle_t queue) {
    ActuatorCmd_t cmd;
    cmd.header    = ACTUATOR_CMD_HEADER;
    cmd.servo_pan = fireXToServoPan(fireX);
    cmd.pump_on   = pumpOn ? 1 : 0;
    cmd.buzzer_on = buzzerOn ? 1 : 0;
    cmd.crc8      = 0; // CRC se duoc tinh boi SlaveComm::sendCommand

    // Day vao queue — Task_UART se gui xuong WROOM
    xQueueOverwrite(queue, &cmd);
}
