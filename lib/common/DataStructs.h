#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H

#include <stdint.h>

// ==========================================
// ENUM TRANG THAI ROBOT (State Machine)
// Theo spec: Tai_Lieu_So_3, Muc 4
// ==========================================
typedef enum {
    STATE_IDLE      = 0,  // Cho ket noi micro-ROS, PWM = 0
    STATE_AUTO      = 1,  // Chay theo quy dao Nav2 (/cmd_vel)
    STATE_MANUAL    = 2,  // Chay theo lenh teleop tu Laptop
    STATE_EMERGENCY = 3   // Mat ket noi > 1000ms, phanh cung + buzzer
} RobotState_t;

// ==========================================
// DU LIEU VAN TOC 4 BANH
// ==========================================
typedef struct {
    float v_fl; // Front Left  (rad/s)
    float v_fr; // Front Right (rad/s)
    float v_rl; // Rear Left   (rad/s)
    float v_rr; // Rear Right  (rad/s)
} MotorSpeeds_t;

// ==========================================
// GOI TIN WROOM -> S3 (Cam bien moi truong)
// Theo spec: Tai_Lieu_So_3, Muc 3.2
// Header: 0xAA | Tong: 14 bytes
// ==========================================
typedef struct __attribute__((packed)) {
    uint8_t  header;        // 0xAA
    uint8_t  fire_flags;    // Bit 0-2: 3 cam bien lua IR (1=co lua)
    float    gas_ppm;       // Nong do gas (da loc EMA)
    float    temperature;   // Nhiet do (°C)
    float    batt_voltage;  // Dien ap pin (V)
    uint8_t  crc8;          // Checksum CRC8
} SensorPacket_t;

// ==========================================
// GOI TIN S3 -> WROOM (Lenh dieu khien)
// Theo spec: Tai_Lieu_So_3, Muc 3.2
// Header: 0xBB | Tong: 5 bytes
// ==========================================
typedef struct __attribute__((packed)) {
    uint8_t header;      // 0xBB
    uint8_t servo_pan;   // Goc xoay ngang voi phun (0-180, 90 = center)
    uint8_t pump_on;     // 0: Tat, 1: Bat bom
    uint8_t buzzer_on;   // 0: Tat, 1: Bat buzzer
    uint8_t crc8;        // Checksum CRC8
} ActuatorCmd_t;

// ==========================================
// DU LIEU ODOMETRY (Vi tri uoc luong)
// ==========================================
typedef struct {
    float x;                // Toa do X (m)
    float y;                // Toa do Y (m)
    float yaw;              // Goc heading (rad)
    float linear_velocity;  // Van toc tuyen tinh (m/s)
    float angular_velocity; // Van toc goc (rad/s)
} OdometryData_t;

// ==========================================
// TRANG THAI MOTION (Chia se giua Task)
// Core1 ghi, Core0 doc de log/publish
// ==========================================
typedef struct {
    float w_fl, w_rl, w_fr, w_rr;           // Van toc thuc te (rad/s)
    float tgt_fl, tgt_rl, tgt_fr, tgt_rr;   // Muc tieu (rad/s)
    float pwm_fl, pwm_rl, pwm_fr, pwm_rr;   // PWM cuoi cung
    int64_t enc_fl, enc_rl, enc_fr, enc_rr;  // Raw encoder count
    float theta;                             // Goc heading (rad)
    float x;                                 // Toa do X (m)
    float y;                                 // Toa do Y (m)
    float alpha;                             // He so alpha adaptive filter (debug)
    float gyro_z;                            // Raw gyro_z tu IMU (rad/s) — debug drift
} MotionState_t;

// ==========================================
// LENH DIEU KHIEN (Tu micro-ROS /cmd_vel hoac Web)
// Core0 ghi (subscriber), Core1 doc (motion)
// ==========================================
typedef struct {
    float target_vx;     // Van toc tuyen tinh muc tieu (m/s)
    float target_wz;     // Van toc goc muc tieu (rad/s)
} CmdVel_t;

// ==========================================
// PROTOCOL CONSTANTS
// ==========================================
#define SENSOR_PACKET_HEADER  0xAA
#define ACTUATOR_CMD_HEADER   0xBB

#endif // DATA_STRUCTS_H
