#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "../common/PinConfig.h"

class IMUDriver {
public:
    IMUDriver();
    bool init();
    void update(float dt);
    
    float getPitch();
    float getRoll();
    // True only when a complete, fresh I2C sample was read.
    bool tryGetGyroZ(float& gyro_z);

private:
    Adafruit_MPU6050 mpu;
    
    float pitch;
    float roll;
    float gyroZ_bias;
    float pitch_bias;
    float roll_bias;
    
    struct RawSample {
        int16_t accel_x, accel_y, accel_z;
        int16_t gyro_x, gyro_y, gyro_z;
    };
    bool readRawSample(RawSample& sample);
    bool calibrateGyro();
    bool ready = false;
};

extern IMUDriver imuDriver;

#endif
