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
    float getGyroZ(); // raw angular velocity

private:
    Adafruit_MPU6050 mpu;
    
    float pitch;
    float roll;
    float gyroZ_bias;
    float pitch_bias;
    float roll_bias;
    
    void calibrateGyro();
};

extern IMUDriver imuDriver;

#endif
