#pragma once

#include "DataStructs.h"
#include <stdint.h>

/**
 * @brief May trang thai chinh cua Robot.
 *
 * Quan ly 4 trang thai: IDLE -> AUTO/MANUAL -> EMERGENCY
 * Thread-safe: dung volatile + atomic cho currentState.
 *
 * Logic chuyen trang thai:
 *   - Boot              -> STATE_IDLE
 *   - Co ket noi + cmd  -> STATE_AUTO hoac STATE_MANUAL
 *   - Mat ket noi > 1s  -> STATE_EMERGENCY
 *   - Ket noi lai       -> STATE_IDLE
 */
class RobotMaster {
public:
    RobotMaster();

    /**
     * @brief Khoi tao State Machine, set IDLE.
     */
    void init();

    /**
     * @brief Goi moi cycle de cap nhat trang thai.
     * Kiem tra timeout cmd_vel -> chuyen EMERGENCY neu can.
     */
    void update();

    /**
     * @brief Lay trang thai hien tai (thread-safe).
     */
    RobotState_t getState() const;

    /**
     * @brief Dat trang thai moi (thread-safe).
     * In log khi chuyen trang thai.
     */
    void setState(RobotState_t newState);

    /**
     * @brief Goi khi nhan duoc /cmd_vel hop le.
     * Reset watchdog timer.
     */
    void notifyCmdReceived();

    /**
     * @brief Kiem tra robot co duoc phep chay motor khong.
     * Chi cho phep o STATE_AUTO hoac STATE_MANUAL.
     */
    bool isMotionAllowed() const;

private:
    volatile RobotState_t _currentState;
    uint32_t _lastCmdTimeMs;   // Thoi diem nhan cmd_vel gan nhat
    bool _connectionActive;    // Co ket noi micro-ROS (tam dung Web cmd)

    /**
     * @brief Tra ve ten trang thai de in log.
     */
    static const char* stateToString(RobotState_t state);
};
