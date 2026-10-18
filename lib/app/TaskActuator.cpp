#include "ActuatorControl.h"
#include "ActuatorLease.h"
#include "CommandFreshness.h"
#include "PumpOutput.h"
#include "PumpSafety.h"
#include "RobotConfig.h"
#include "RobotMaster.h"
#include "ServoConfig.h"
#include "TaskManager.h"
#if SERVO_SWEEP_ENABLED
#include "ServoOutput.h"
#include "ServoSweep.h"
#endif
#include <Arduino.h>
#include <math.h>

extern RobotMaster robotMaster;

namespace {
portMUX_TYPE s_actuator_mux = portMUX_INITIALIZER_UNLOCKED;

struct ActuatorInput {
    bool pumpOn = false;
    uint32_t pumpMs = 0;
    uint32_t pumpRevision = 0;
    uint32_t offRevision = 0;
    bool hostOnline = false;
    uint32_t serialTickMs = 0;
};

ActuatorInput s_input;
ActuatorStatus s_status;

ActuatorInput inputSnapshot() {
    portENTER_CRITICAL(&s_actuator_mux);
    const ActuatorInput snapshot = s_input;
    portEXIT_CRITICAL(&s_actuator_mux);
    return snapshot;
}

void publishStatus(const ActuatorStatus& status) {
    portENTER_CRITICAL(&s_actuator_mux);
    s_status = status;
    portEXIT_CRITICAL(&s_actuator_mux);
}
} // namespace

void actuatorSubmitPump(bool on, uint32_t received_ms) {
    portENTER_CRITICAL(&s_actuator_mux);
    s_input.pumpOn = on;
    s_input.pumpMs = received_ms;
    ++s_input.pumpRevision;
    if (!on) ++s_input.offRevision;
    portEXIT_CRITICAL(&s_actuator_mux);
}

void actuatorPublishLink(bool online, uint32_t tick_ms) {
    portENTER_CRITICAL(&s_actuator_mux);
    s_input.hostOnline = online;
    s_input.serialTickMs = tick_ms;
    portEXIT_CRITICAL(&s_actuator_mux);
}

ActuatorStatus actuatorStatusSnapshot() {
    portENTER_CRITICAL(&s_actuator_mux);
    const ActuatorStatus snapshot = s_status;
    portEXIT_CRITICAL(&s_actuator_mux);
    return snapshot;
}

void Task_Actuator(void* pvParam) {
    SharedContext* ctx = static_cast<SharedContext*>(pvParam);
    PumpSafety pumpSafety;
#if SERVO_SWEEP_ENABLED
    ServoSweep sweep(SERVO_SWEEP_START_DEG, SERVO_SWEEP_TURN_DEG,
                     SERVO_SWEEP_END_DEG, SERVO_SWEEP_STEP_PERIOD_MS);
    bool servoFault = false;
    uint32_t homeCompletions = 0;
#endif
    uint32_t seenPumpRevision = 0;
    uint32_t seenOffRevision = 0;
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TASK_SERIALCOMM_PERIOD_MS));
        const uint32_t now = millis();
        const ActuatorInput input = inputSnapshot();
        const char* eventLog = nullptr;

        // A short OFF->ON burst within one actuator period did not actually
        // turn the PWM off, so it must not reset the continuous-run cap.
        if (input.offRevision != seenOffRevision) {
            if (actuatorObservedOff(input.offRevision, seenOffRevision,
                                    input.pumpOn)) pumpSafety.request(false, now);
            seenOffRevision = input.offRevision;
        }
        if (input.pumpRevision != seenPumpRevision) {
            if (input.pumpOn) pumpSafety.request(true, input.pumpMs);
            seenPumpRevision = input.pumpRevision;
        }

        bool commandFresh = false;
        bool driveZero = false;
        if (xSemaphoreTake(ctx->cmdMutex, 0) == pdTRUE) {
            const float vx = ctx->cmdVel.target_vx;
            const float wz = ctx->cmdVel.target_wz;
            commandFresh = commandIsFresh(ctx->hasValidCmd,
                ctx->lastCmdReceivedMs, now, CMD_VEL_TIMEOUT_MS) &&
                isfinite(vx) && isfinite(wz);
            driveZero = commandFresh && fabsf(vx) < 0.001f &&
                fabsf(wz) < 0.001f;
            xSemaphoreGive(ctx->cmdMutex);
        }

        const bool linkOk = actuatorSerialLinkFresh(
            input.hostOnline, input.serialTickMs, now) &&
            commandFresh && robotMaster.isMotionAllowed();
        const bool requestAllowed = pumpSafety.tick(now, linkOk);
#if SERVO_SWEEP_ENABLED
        const bool safeToHome = linkOk && driveZero;
        if (sweep.returningHome() && !safeToHome) {
            sweep.abort();
            servoOutput.stop();
            eventLog = "[ERROR][SERVO] Home interrupted; pump locked until reboot.";
        }
        if (sweep.active() && !requestAllowed) {
            const bool normalStop = !input.pumpOn ||
                pumpSafety.stopReason() == PumpSafety::StopReason::MAX_RUN;
            if (normalStop && safeToHome && sweep.beginReturn(now)) {
                // Wait for the bounded return before another sweep.
            } else {
                sweep.abort();
                servoOutput.stop();
                eventLog = "[ERROR][SERVO] Sweep interrupted; pump locked until reboot.";
            }
        }
        if (sweep.returningHome() && !input.pumpOn && safeToHome &&
            sweep.rearm(now)) {
            ++homeCompletions;
            eventLog = "[SERVO] Nozzle at 90 deg; new OFF->ON cycle allowed.";
        }
        const bool pumpAllowed = requestAllowed && !sweep.consumed() &&
            !servoFault;
#else
        const bool pumpAllowed = requestAllowed;
#endif
        pumpOutput.setEnabled(pumpAllowed);

        ActuatorStatus status;
        status.pumpReady = pumpOutput.ready();
#if SERVO_SWEEP_ENABLED
        const int nozzleAngle = sweep.update(
            pumpAllowed && status.pumpReady, now);
        if (nozzleAngle < 0) {
            if (servoOutput.active()) servoOutput.stop();
            if (pumpAllowed) {
                servoFault = true;
                sweep.abort();
                pumpOutput.setEnabled(false);
                eventLog = "[ERROR][SERVO] Sweep unavailable; pump OFF until reboot.";
            }
        } else if (!servoOutput.writeAngle(nozzleAngle)) {
            servoFault = true;
            sweep.abort();
            pumpOutput.setEnabled(false);
            servoOutput.stop();
            eventLog = "[ERROR][SERVO] PWM write failed; pump and servo OFF until reboot.";
        }
        status.servoActive = servoOutput.active();
        status.phase = (servoFault ||
            (sweep.consumed() && !sweep.returningHome())) ? 3 :
            sweep.returningHome() ? 2 : sweep.active() ? 1 : 0;
        status.angle = status.servoActive && nozzleAngle >= 0 ?
            static_cast<uint8_t>(nozzleAngle) : 255;
        status.homeCompletions = homeCompletions;
#endif
        status.pumpOn = pumpOutput.commandedOn();
        publishStatus(status);
        if (eventLog != nullptr) DBG.println(eventLog);
    }
}
