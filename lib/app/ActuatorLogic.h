#pragma once

#include "DataStructs.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

/**
 * @brief Xu ly toa do lua tu YOLO -> tinh goc Servo Pan.
 *
 * Nhan toa do tam lua chuan hoa [-1.0, 1.0] tu /fire_target,
 * chuyen doi thanh goc servo (0-180), dong goi ActuatorCmd_t
 * roi day vao actuatorQueue de Task_UART gui xuong WROOM.
 */
class ActuatorLogic {
public:
    /**
     * @brief Xu ly toa do lua va gui lenh servo + bom.
     * @param fireX   Toa do ngang cua lua [-1.0, 1.0] (0 = giua)
     * @param pumpOn  Bat/tat bom nuoc
     * @param buzzerOn Bat/tat buzzer
     * @param queue   actuatorQueue de gui lenh xuong WROOM
     */
    static void processFireTarget(float fireX, bool pumpOn, bool buzzerOn,
                                  QueueHandle_t queue);

    /**
     * @brief Chuyen toa do lua [-1, 1] thanh goc servo [0, 180].
     * -1.0 = 0° (ben trai), 0.0 = 90° (giua), 1.0 = 180° (ben phai)
     */
    static uint8_t fireXToServoPan(float fireX);
};
