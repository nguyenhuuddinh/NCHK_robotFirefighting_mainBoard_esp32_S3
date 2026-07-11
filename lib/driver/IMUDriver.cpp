#include "IMUDriver.h"
#include "RobotConfig.h"

IMUDriver imuDriver;

IMUDriver::IMUDriver() : pitch(0), roll(0), gyroZ_bias(0), pitch_bias(0), roll_bias(0) {}

bool IMUDriver::init() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000); // 400kHz I2C
    
    if (!mpu.begin()) {
        DBG.println("[ERROR] Không tìm thấy MPU6050!");
        return false;
    }
    
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 
    
    calibrateGyro();
    
    pitch = 0;
    roll = 0;
    return true;
}

void IMUDriver::calibrateGyro() {
    DBG.println("[INFO] Đang hiệu chỉnh IMU (Gyro & Accel). Vui lòng giữ yên xe...");
    float sumZ = 0;
    float sumPitch = 0;
    float sumRoll = 0;
    int samples = 500;
    for (int i = 0; i < samples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        sumZ += g.gyro.z;
        
        // Tính toán Pitch và Roll tức thời
        float r = atan2(a.acceleration.y, a.acceleration.z);
        float p = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z));
        
        sumRoll += r;
        sumPitch += p;
        
        vTaskDelay(pdMS_TO_TICKS(3)); // Khong blocking
    }
    gyroZ_bias = sumZ / samples;
    pitch_bias = sumPitch / samples;
    roll_bias = sumRoll / samples;
    
    DBG.print("[INFO] Gyro Z Bias: "); DBG.print(gyroZ_bias, 6);
    DBG.print(" | Pitch Bias: "); DBG.print(pitch_bias, 6);
    DBG.print(" | Roll Bias: "); DBG.println(roll_bias, 6);
}

void IMUDriver::update(float float_dt) {
    if (float_dt <= 0.0f) return;
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Cập nhật PITCH và ROLL (Có trừ đi góc sai số ban đầu - bias)
    float raw_roll = atan2(a.acceleration.y, a.acceleration.z) - roll_bias;
    float raw_pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) - pitch_bias;
    
    // Áp dụng Bộ Lọc Bù (Complementary Filter):
    // 96% tin tưởng vào Gyro (mượt, phản hồi nhanh nhưng bị trôi dài hạn)
    // 4% tin tưởng vào Accelerometer (nhiễu nhiều nhưng cho góc tuyệt đối đúng so với trọng lực)
    roll = 0.96f * (roll + g.gyro.x * float_dt) + 0.04f * raw_roll;
    pitch = 0.96f * (pitch + g.gyro.y * float_dt) + 0.04f * raw_pitch;
}

float IMUDriver::getPitch() { return pitch; }
float IMUDriver::getRoll() { return roll; }

float IMUDriver::getGyroZ() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    return g.gyro.z - gyroZ_bias;
}
