// Isolated, opt-in hardware diagnostic for ESP32-S3 N8R2.
// This file is built only by the *_actuator_test PlatformIO environment.
// It does not start the ROS serial bridge, motion tasks, or motor PWM.
#if defined(ACTUATOR_TEST_MODE) && ACTUATOR_TEST_MODE
#include <Arduino.h>
#include <string.h>

#include "PinConfig.h"
#include "ServoConfig.h"
#include "PumpOutput.h"
#include "ServoOutput.h"
#include "../lib/service/ServoSweep.h"

namespace {

constexpr uint32_t kPumpLimitMs = 5000;
constexpr uint32_t kProgressPeriodMs = 500;

enum class Phase : uint8_t { IDLE, RUNNING, HOMING, WAIT_OFF, FAULT };

ServoSweep sweep(SERVO_SWEEP_START_DEG, SERVO_SWEEP_TURN_DEG,
                 SERVO_SWEEP_END_DEG, SERVO_SWEEP_STEP_PERIOD_MS);
Phase phase = Phase::IDLE;
bool off_seen = false;
uint32_t run_started_ms = 0;
uint32_t last_progress_ms = 0;
char line[24];
size_t line_len = 0;

void fault(const char* reason) {
    pumpOutput.setEnabled(false);
    servoOutput.stop();
    sweep.abort();
    phase = Phase::FAULT;
    off_seen = false;
    Serial0.printf("[ERROR][ACT-TEST] %s; GPIO47 and GPIO38 OFF. Reboot before retry.\n",
                   reason);
}

void command(const char* text) {
    if (strcmp(text, "help") == 0) {
        Serial0.println("[ACT-TEST] Commands: status, off, run, help");
        Serial0.println("[ACT-TEST] Send off, then run. Pump power DISCONNECTED for first test.");
        return;
    }
    if (strcmp(text, "status") == 0) {
        const char* name = phase == Phase::IDLE ? "IDLE" :
                           phase == Phase::RUNNING ? "RUNNING" :
                           phase == Phase::HOMING ? "HOMING" :
                           phase == Phase::WAIT_OFF ? "WAIT_OFF" : "FAULT";
        Serial0.printf("[ACT-TEST] state=%s off_seen=%d pump_ready=%d servo_active=%d\n",
                       name, off_seen, pumpOutput.ready(), servoOutput.active());
        return;
    }
    if (strcmp(text, "off") == 0) {
        pumpOutput.setEnabled(false);
        if (phase == Phase::RUNNING || phase == Phase::HOMING) {
            fault("operator OFF interrupted the trajectory");
            return;
        }
        servoOutput.stop();
        if (phase == Phase::FAULT) {
            Serial0.println("[ACT-TEST] OFF acknowledged; fault remains latched until reboot.");
            return;
        }
        if (phase == Phase::WAIT_OFF) {
            if (!sweep.rearm(millis())) {
                fault("home settle proof missing");
                return;
            }
            phase = Phase::IDLE;
        }
        off_seen = true;
        Serial0.println("[ACT-TEST] GPIO47/GPIO38 OFF; next run armed.");
        return;
    }
    if (strcmp(text, "run") == 0) {
        if (phase != Phase::IDLE || !off_seen) {
            Serial0.println("[ACT-TEST] Rejected: wait for home, then send off before run.");
            return;
        }
        if (!pumpOutput.ready()) {
            fault("pump PWM GPIO47 not initialized");
            return;
        }
        // The first absolute 90-degree command is only safe when the nozzle
        // has already been physically parked near 90 degrees by the operator.
        if (!servoOutput.writeAngle(SERVO_SWEEP_START_DEG)) {
            fault("servo PWM GPIO38 could not attach");
            return;
        }
        const uint32_t now = millis();
        if (sweep.update(true, now) < 0) {
            fault("servo sweep did not start");
            return;
        }
        off_seen = false;
        run_started_ms = now;
        last_progress_ms = now;
        phase = Phase::RUNNING;
        pumpOutput.setEnabled(true);
        Serial0.println("[ACT-TEST] RUN: GPIO47 ON; GPIO38 90 -> 45 -> 115. Hard stop 5 s.");
        return;
    }
    Serial0.println("[ACT-TEST] Rejected; type help. No output changed.");
}

void pollCommands() {
    uint8_t budget = 32;
    while (budget-- > 0 && Serial0.available() > 0) {
        const char c = static_cast<char>(Serial0.read());
        if (c == '\r' || c == '\n') {
            if (line_len > 0) {
                line[line_len] = '\0';
                command(line);
                line_len = 0;
            }
        } else if (line_len < sizeof(line) - 1) {
            line[line_len++] = c;
        } else {
            line_len = 0;
            fault("serial command too long");
        }
    }
}

void updateActuators() {
    const uint32_t now = millis();
    if (phase == Phase::RUNNING) {
        if (static_cast<uint32_t>(now - run_started_ms) >= kPumpLimitMs) {
            pumpOutput.setEnabled(false);
            Serial0.println("[ACT-TEST] 5 s cap: GPIO47 OFF.");
            if (!sweep.atEnd() || !sweep.beginReturn(now)) {
                fault("sweep did not reach 115 before pump cap");
                return;
            }
            phase = Phase::HOMING;
            Serial0.println("[ACT-TEST] Returning GPIO38 to 90 degrees.");
        } else {
            const int angle = sweep.update(true, now);
            if (angle < 0 || !servoOutput.writeAngle(angle)) {
                fault("servo PWM failed during sweep");
                return;
            }
            if (static_cast<uint32_t>(now - last_progress_ms) >= kProgressPeriodMs) {
                Serial0.printf("[ACT-TEST] elapsed=%lu ms angle=%d pump=ON\n",
                               static_cast<unsigned long>(now - run_started_ms), angle);
                last_progress_ms = now;
            }
        }
    }
    if (phase == Phase::HOMING) {
        const int angle = sweep.update(false, now);
        if (angle < 0 || !servoOutput.writeAngle(angle)) {
            fault("servo PWM failed during return");
            return;
        }
        if (sweep.returnSettled(now)) {
            servoOutput.stop();
            phase = Phase::WAIT_OFF;
            Serial0.println("[ACT-TEST] GPIO38 at commanded 90; PWM OFF. Send off, then run again.");
        }
    }
}

} // namespace

void setup() {
    // The test firmware must never energize a motor, even when the Pi sends
    // a stale command over the native USB port (which this build ignores).
    const uint8_t motor_pins[] = {
        MOT_ENA_FL_PIN, MOT_ENB_RL_PIN, MOT_ENA_FR_PIN, MOT_ENB_RR_PIN,
        MOT_IN1_FL_PIN, MOT_IN2_FL_PIN, MOT_IN3_RL_PIN, MOT_IN4_RL_PIN,
        MOT_IN1_FR_PIN, MOT_IN2_FR_PIN, MOT_IN3_RR_PIN, MOT_IN4_RR_PIN};
    for (uint8_t pin : motor_pins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    pinMode(PUMP_PWM_PIN, OUTPUT);
    digitalWrite(PUMP_PWM_PIN, LOW);
    servoOutput.init();
    Serial0.begin(115200);
    if (!pumpOutput.init()) fault("pump PWM GPIO47 init failed");
    Serial0.println("[ACT-TEST] Isolated diagnostic firmware; motors OFF, no ROS bridge.");
    Serial0.println("[ACT-TEST] GPIO47/GPIO38 OFF at boot. Park nozzle near 90 degrees.");
    command("help");
}

void loop() {
    pollCommands();
    updateActuators();
    vTaskDelay(pdMS_TO_TICKS(10));
}
#endif // ACTUATOR_TEST_MODE
