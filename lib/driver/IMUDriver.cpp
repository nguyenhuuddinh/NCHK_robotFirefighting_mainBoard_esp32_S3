#include "IMUDriver.h"
#include "RobotConfig.h"

IMUDriver imuDriver;

IMUDriver::IMUDriver() : pitch(0), roll(0), gyroZ_bias(0), pitch_bias(0), roll_bias(0) {}

bool IMUDriver::init() {
    ready = false;
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000); // 400kHz I2C
    Wire.setTimeOut(5); // Bound a stalled shared I2C transaction.
    
    if (!mpu.begin()) {
        DBG.println("[ERROR] Không tìm thấy MPU6050!");
        return false;
    }
    
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 
    
    if (!calibrateGyro()) {
        DBG.println("[ERROR] MPU6050 calibration read failed; encoder-only until reboot.");
        return false;
    }
    
    pitch = 0;
    roll = 0;
    ready = true;
    return true;
}

bool IMUDriver::readRawSample(RawSample& sample) {
    // Adafruit_MPU6050::getEvent() discards the I2C read result. Read the
    // 14-byte burst directly so a partial/failed transfer cannot become gyro.
    Wire.beginTransmission(MPU6050_I2CADDR_DEFAULT);
    if (Wire.write((uint8_t)MPU6050_ACCEL_OUT) != 1 ||
        Wire.endTransmission(false) != 0) {
        return false;
    }
    if (Wire.requestFrom((uint8_t)MPU6050_I2CADDR_DEFAULT,
                         (uint8_t)14, (uint8_t)true) != 14) {
        return false;
    }
    uint8_t raw[14];
    for (uint8_t i = 0; i < 14; ++i) {
        const int value = Wire.read();
        if (value < 0) return false;
        raw[i] = (uint8_t)value;
    }
    auto signed16 = [&](uint8_t index) -> int16_t {
        return (int16_t)(((uint16_t)raw[index] << 8) | raw[index + 1]);
    };
    sample.accel_x = signed16(0);
    sample.accel_y = signed16(2);
    sample.accel_z = signed16(4);
    sample.gyro_x = signed16(8);
    sample.gyro_y = signed16(10);
    sample.gyro_z = signed16(12);
    return true;
}

bool IMUDriver::calibrateGyro() {
    DBG.println("[INFO] Đang hiệu chỉnh IMU (Gyro & Accel). Vui lòng giữ yên xe...");
    float sumZ = 0;
    float sumPitch = 0;
    float sumRoll = 0;
    const int samples = 500;
    int valid_samples = 0;
    for (int attempts = 0; attempts < samples + 50 && valid_samples < samples;
         ++attempts) {
        RawSample raw;
        if (!readRawSample(raw)) {
            vTaskDelay(pdMS_TO_TICKS(3));
            continue;
        }
        ++valid_samples;
        sumZ += (float)raw.gyro_z * (DEG_TO_RAD / 131.0f);
        
        // Tính toán Pitch và Roll tức thời
        float r = atan2f((float)raw.accel_y, (float)raw.accel_z);
        float p = atan2f(-(float)raw.accel_x,
                         sqrtf((float)raw.accel_y * raw.accel_y +
                               (float)raw.accel_z * raw.accel_z));
        
        sumRoll += r;
        sumPitch += p;
        
        vTaskDelay(pdMS_TO_TICKS(3)); // Khong blocking
    }
    if (valid_samples != samples) return false;
    gyroZ_bias = sumZ / samples;
    pitch_bias = sumPitch / samples;
    roll_bias = sumRoll / samples;
    
    DBG.print("[INFO] Gyro Z Bias: "); DBG.print(gyroZ_bias, 6);
    DBG.print(" | Pitch Bias: "); DBG.print(pitch_bias, 6);
    DBG.print(" | Roll Bias: "); DBG.println(roll_bias, 6);
    return true;
}

void IMUDriver::update(float float_dt) {
    if (float_dt <= 0.0f) return;
    
    if (!ready) return;
    RawSample raw;
    if (!readRawSample(raw)) return;
    
    // Cập nhật PITCH và ROLL (Có trừ đi góc sai số ban đầu - bias)
    float raw_roll = atan2f((float)raw.accel_y, (float)raw.accel_z) - roll_bias;
    float raw_pitch = atan2f(-(float)raw.accel_x,
                             sqrtf((float)raw.accel_y * raw.accel_y +
                                   (float)raw.accel_z * raw.accel_z)) - pitch_bias;
    
    // Áp dụng Bộ Lọc Bù (Complementary Filter):
    // 96% tin tưởng vào Gyro (mượt, phản hồi nhanh nhưng bị trôi dài hạn)
    // 4% tin tưởng vào Accelerometer (nhiễu nhiều nhưng cho góc tuyệt đối đúng so với trọng lực)
    roll = 0.96f * (roll + raw.gyro_x * (DEG_TO_RAD / 131.0f) * float_dt) + 0.04f * raw_roll;
    pitch = 0.96f * (pitch + raw.gyro_y * (DEG_TO_RAD / 131.0f) * float_dt) + 0.04f * raw_pitch;
}

float IMUDriver::getPitch() { return pitch; }
float IMUDriver::getRoll() { return roll; }

bool IMUDriver::tryGetGyroZ(float& gyro_z) {
    if (!ready) return false;
    RawSample raw;
    if (!readRawSample(raw)) return false;
    gyro_z = (float)raw.gyro_z * (DEG_TO_RAD / 131.0f) - gyroZ_bias;
    return true;
}
