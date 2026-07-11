#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "../common/DataStructs.h"

class Odometry {
public:
    Odometry();
    void init();
    
    // Cập nhật vị trí (Odometry) và Sensor Fusion
    // - v_x: Vận tốc tịnh tiến tính từ Encoder (m/s)
    // - wz_encoder: Vận tốc góc từ Encoder (rad/s)
    // - gyro_z: Vận tốc góc từ IMU (rad/s)
    // - dt: thời gian trôi qua (giây)
    void update(float v_x, float wz_encoder, float gyro_z, float dt);
    
    OdometryData_t getOdometry();
    
    // Lay gia tri alpha hien tai (dung cho Logger)
    float getLastAlpha();

private:
    OdometryData_t odom;
    float last_alpha;  // Gia tri alpha cuoi cung duoc su dung (adaptive)
};

extern Odometry odometry;

#endif
