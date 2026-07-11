#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include <Arduino.h>

// ==========================================
// 1. ENCODER PINS
// ==========================================
// Motor 1 (Left Front)
#define ENC1_A_PIN 8
#define ENC1_B_PIN 9

// Motor 2 (Left Rear)
#define ENC2_A_PIN 10
#define ENC2_B_PIN 11

// Motor 3 (Right Front)
#define ENC3_A_PIN 12
#define ENC3_B_PIN 13

// Motor 4 (Right Rear)
#define ENC4_A_PIN 14
#define ENC4_B_PIN 15

// ==========================================
// 2. MOTOR DRIVER PINS
// ==========================================
// Left Side (Motor 1 & 2)
#define MOT_IN1_FL_PIN 4  // Dir Motor 1
#define MOT_IN2_FL_PIN 5  // Dir Motor 1
#define MOT_IN3_RL_PIN 6  // Dir Motor 2
#define MOT_IN4_RL_PIN 7  // Dir Motor 2
#define MOT_ENA_FL_PIN 16 // PWM Motor 1
#define MOT_ENB_RL_PIN 17 // PWM Motor 2

// Right Side (Motor 3 & 4)
#define MOT_IN1_FR_PIN 18 // Dir Motor 3
#define MOT_IN2_FR_PIN 21 // Dir Motor 3
#define MOT_IN3_RR_PIN 39 // Dir Motor 4
#define MOT_IN4_RR_PIN 40 // Dir Motor 4
#define MOT_ENA_FR_PIN 41 // PWM Motor 3
#define MOT_ENB_RR_PIN 42 // PWM Motor 4

// ==========================================
// 3. I2C & SENSORS PINS
// ==========================================
#define I2C_SCL_PIN 1
#define I2C_SDA_PIN 2

// ==========================================
// 4. ACTUATOR PINS (Da chuyen sang WROOM)
// ==========================================
// Servo Pan/Tilt da chuyen sang ESP32-WROOM dieu khien qua UART
// GPIO 3, 46 con trong — du phong tuong lai

// ==========================================
// 5. UART (WROOM COMMUNICATION) PINS
// ==========================================
#define SLAVE_RX_PIN 20
#define SLAVE_TX_PIN 19

#endif // PIN_CONFIG_H
