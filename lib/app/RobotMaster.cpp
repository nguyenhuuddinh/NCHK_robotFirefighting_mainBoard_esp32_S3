#include "RobotMaster.h"
#include "RobotConfig.h"
#include <Arduino.h>

RobotMaster::RobotMaster()
    : _currentState(STATE_IDLE)
    , _lastCmdTimeMs(0)
    , _connectionActive(false) {}

void RobotMaster::init() {
    _currentState = STATE_IDLE;
    _lastCmdTimeMs = millis();
    _connectionActive = false;
    DBG.println("[STATE] RobotMaster initialized -> IDLE");
}

void RobotMaster::update() {
    uint32_t now = millis();
    RobotState_t current = _currentState;

    switch (current) {
        case STATE_IDLE:
            // Cho lenh dieu khien, khong lam gi dac biet
            break;

        case STATE_AUTO:
        case STATE_MANUAL:
            // Kiem tra watchdog: mat cmd_vel > timeout -> EMERGENCY
            if ((now - _lastCmdTimeMs) > CMD_VEL_TIMEOUT_MS) {
                setState(STATE_EMERGENCY);
                DBG.printf("[STATE] Watchdog timeout! No cmd_vel for %dms\n",
                              CMD_VEL_TIMEOUT_MS);
            }
            break;

        case STATE_EMERGENCY:
            // O EMERGENCY cho den khi nhan lai cmd -> chuyen IDLE
            // (Se duoc goi tu notifyCmdReceived)
            break;
    }
}

RobotState_t RobotMaster::getState() const {
    return _currentState;
}

void RobotMaster::setState(RobotState_t newState) {
    RobotState_t old = _currentState;
    if (old == newState) return; // Khong log neu khong doi

    _currentState = newState;
    DBG.printf("[STATE] %s -> %s\n",
                  stateToString(old), stateToString(newState));
}

void RobotMaster::notifyCmdReceived() {
    _lastCmdTimeMs = millis();

    // Neu dang IDLE hoac EMERGENCY -> chuyen sang MANUAL (tam thoi)
    // Sau nay khi co micro-ROS se phan biet AUTO vs MANUAL
    RobotState_t current = _currentState;
    if (current == STATE_IDLE || current == STATE_EMERGENCY) {
        setState(STATE_MANUAL);
    }
}

bool RobotMaster::isMotionAllowed() const {
    RobotState_t s = _currentState;
    return (s == STATE_AUTO || s == STATE_MANUAL);
}

const char* RobotMaster::stateToString(RobotState_t state) {
    switch (state) {
        case STATE_IDLE:      return "IDLE";
        case STATE_AUTO:      return "AUTO";
        case STATE_MANUAL:    return "MANUAL";
        case STATE_EMERGENCY: return "EMERGENCY";
        default:              return "UNKNOWN";
    }
}
