#include "MotorDriver.h"

MotorDriver motorDriver;

MotorDriver::MotorDriver() {}

void MotorDriver::init() {
    // Front Left (Motor 1) - Driver L298N #1 Trái
    pinMode(MOT_IN1_FL_PIN, OUTPUT);
    pinMode(MOT_IN2_FL_PIN, OUTPUT);
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ledcAttach(MOT_ENA_FL_PIN, pwmFreq, pwmResolution);
    #else
    ledcSetup(chFL, pwmFreq, pwmResolution);
    ledcAttachPin(MOT_ENA_FL_PIN, chFL);
    #endif

    // Rear Left (Motor 2) - Driver L298N #1 Phải
    pinMode(MOT_IN3_RL_PIN, OUTPUT);
    pinMode(MOT_IN4_RL_PIN, OUTPUT);
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ledcAttach(MOT_ENB_RL_PIN, pwmFreq, pwmResolution);
    #else
    ledcSetup(chRL, pwmFreq, pwmResolution);
    ledcAttachPin(MOT_ENB_RL_PIN, chRL);
    #endif

    // Front Right (Motor 3) - Driver L298N #2 Trái
    pinMode(MOT_IN1_FR_PIN, OUTPUT);
    pinMode(MOT_IN2_FR_PIN, OUTPUT);
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ledcAttach(MOT_ENA_FR_PIN, pwmFreq, pwmResolution);
    #else
    ledcSetup(chFR, pwmFreq, pwmResolution);
    ledcAttachPin(MOT_ENA_FR_PIN, chFR);
    #endif

    // Rear Right (Motor 4) - Driver L298N #2 Phải
    pinMode(MOT_IN3_RR_PIN, OUTPUT);
    pinMode(MOT_IN4_RR_PIN, OUTPUT);
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ledcAttach(MOT_ENB_RR_PIN, pwmFreq, pwmResolution);
    #else
    ledcSetup(chRR, pwmFreq, pwmResolution);
    ledcAttachPin(MOT_ENB_RR_PIN, chRR);
    #endif

    stopAll();
}

void MotorDriver::setMotorFL(int speed) {
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        digitalWrite(MOT_IN1_FL_PIN, HIGH);
        digitalWrite(MOT_IN2_FL_PIN, LOW);
    } else if (speed < 0) {
        digitalWrite(MOT_IN1_FL_PIN, LOW);
        digitalWrite(MOT_IN2_FL_PIN, HIGH);
        speed = -speed;
    } else {
        digitalWrite(MOT_IN1_FL_PIN, LOW);
        digitalWrite(MOT_IN2_FL_PIN, LOW);
    }
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ledcWrite(MOT_ENA_FL_PIN, speed);
    #else
    ledcWrite(chFL, speed);
    #endif
}

void MotorDriver::setMotorRL(int speed) {
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        digitalWrite(MOT_IN3_RL_PIN, HIGH);
        digitalWrite(MOT_IN4_RL_PIN, LOW);
    } else if (speed < 0) {
        digitalWrite(MOT_IN3_RL_PIN, LOW);
        digitalWrite(MOT_IN4_RL_PIN, HIGH);
        speed = -speed;
    } else {
        digitalWrite(MOT_IN3_RL_PIN, LOW);
        digitalWrite(MOT_IN4_RL_PIN, LOW);
    }
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ledcWrite(MOT_ENB_RL_PIN, speed);
    #else
    ledcWrite(chRL, speed);
    #endif
}

void MotorDriver::setMotorFR(int speed) {
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        digitalWrite(MOT_IN1_FR_PIN, HIGH);
        digitalWrite(MOT_IN2_FR_PIN, LOW);
    } else if (speed < 0) {
        digitalWrite(MOT_IN1_FR_PIN, LOW);
        digitalWrite(MOT_IN2_FR_PIN, HIGH);
        speed = -speed;
    } else {
        digitalWrite(MOT_IN1_FR_PIN, LOW);
        digitalWrite(MOT_IN2_FR_PIN, LOW);
    }
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ledcWrite(MOT_ENA_FR_PIN, speed);
    #else
    ledcWrite(chFR, speed);
    #endif
}

void MotorDriver::setMotorRR(int speed) {
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        digitalWrite(MOT_IN3_RR_PIN, HIGH);
        digitalWrite(MOT_IN4_RR_PIN, LOW);
    } else if (speed < 0) {
        digitalWrite(MOT_IN3_RR_PIN, LOW);
        digitalWrite(MOT_IN4_RR_PIN, HIGH);
        speed = -speed;
    } else {
        digitalWrite(MOT_IN3_RR_PIN, LOW);
        digitalWrite(MOT_IN4_RR_PIN, LOW);
    }
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ledcWrite(MOT_ENB_RR_PIN, speed);
    #else
    ledcWrite(chRR, speed);
    #endif
}

void MotorDriver::setSpeeds(int fl, int rl, int fr, int rr) {
    setMotorFL(fl);
    setMotorRL(rl);
    setMotorFR(fr);
    setMotorRR(rr);
}

void MotorDriver::stopAll() {
    setSpeeds(0, 0, 0, 0);
}
