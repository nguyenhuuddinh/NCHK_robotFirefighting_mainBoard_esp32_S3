#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// ==========================================
// ROBOT PHYSICAL PARAMETERS (SKID-STEER)
// ==========================================
#define WHEEL_RADIUS_M 0.034 // Ban kinh banh xe (m) (Duong kinh 68mm)
#define WHEEL_BASE_M 0.134   // Khoang cach giua 2 banh truoc - sau (m)
#define TRACK_WIDTH_M 0.086  // Khoang cach giua 2 banh trai - phai (m)

// ==========================================
// ENCODER PARAMETERS
// ==========================================
#define ENCODER_PPR 11  // So xung tren 1 vong cua motor (Pulse Per Revolution)
#define GEAR_RATIO 35.5 // Ty so truyen cua hop giam toc (Gear Ratio)
#define ENCODER_TICKS_PER_REV                                                  \
  (ENCODER_PPR * 4.0 * GEAR_RATIO) // So xung / 1 vong banh xe (Full Quad = x4)

// ==========================================
// TUNING STEP - CHI CHINH SO NAY KHI TUNING
// ==========================================
// Buoc 1: Chi PWM_OFFSET (K_FF=0, PID tat)  -> Tim nguong vuot ma sat
// Buoc 2: Them K_FF      (PID tat)           -> Chinh K_FF cho FF bam Tgt
// Buoc 3: Them Kp        (Ki=0)              -> Chinh do nhay P
// Buoc 4: Them Ki        (Full FF+PI)        -> Loai bo steady-state error
#define TUNING_STEP 4

// ==========================================
// PID TARGET VALUES
// Thay doi khi o Buoc 3 (KP) va Buoc 4 (KI)
// Kd = 0: Encoder roi rac -> D khuyech dai nhieu, giu nguyen
// ==========================================
#define PID_KP 10.0
#define PID_KI 0.3
#define PID_KD 0.0

// ==========================================
// FEED-FORWARD PARAMETERS
// (Chuyen tu main.cpp sang day de tap trung quan ly)
// ==========================================
// PWM_OFFSET:      Nguong vuot ma sat KHI DI THANG (ma sat lan - rolling)
// PWM_OFFSET_TURN: Nguong vuot ma sat KHI XOAY TAI CHO (ma sat truot ngang)
// K_FF:            PWM them moi rad/s tang them
// Cong thuc: PWM = sign(omega) * (PWM_OFFSET + |omega| * K_FF)
#define FF_PWM_OFFSET      140.0f  // Chinh o Buoc 1 (di thang)
#define FF_PWM_OFFSET_TURN 140.0f  // Chinh o Buoc 1 (xoay)
#define FF_K_FF             1.5f   // Chinh o Buoc 2

// ==========================================
// COMPUTED TUNING VALUES (Tu dong tinh theo TUNING_STEP)
// KHONG CHINH TRUC TIEP — chi thay doi TUNING_STEP o tren
// ==========================================
#if TUNING_STEP == 1
  #define FF_K_FF_ACTIVE  0.0f
  #define PID_KP_ACTIVE   0.0f
  #define PID_KI_ACTIVE   0.0f
  #define PID_KD_ACTIVE   0.0f
#elif TUNING_STEP == 2
  #define FF_K_FF_ACTIVE  FF_K_FF
  #define PID_KP_ACTIVE   0.0f
  #define PID_KI_ACTIVE   0.0f
  #define PID_KD_ACTIVE   0.0f
#elif TUNING_STEP == 3
  #define FF_K_FF_ACTIVE  FF_K_FF
  #define PID_KP_ACTIVE   PID_KP
  #define PID_KI_ACTIVE   0.0f
  #define PID_KD_ACTIVE   0.0f
#else // TUNING_STEP == 4 (FULL FF + PI)
  #define FF_K_FF_ACTIVE  FF_K_FF
  #define PID_KP_ACTIVE   PID_KP
  #define PID_KI_ACTIVE   PID_KI
  #define PID_KD_ACTIVE   PID_KD
#endif

// Gioi han PWM toi da
#define PWM_MAX 255.0f

// ==========================================
// TASK & RTOS PARAMETERS
// ==========================================
#define CORE_0 0
#define CORE_1 1

#define TASK_MOTION_PERIOD_MS   20   // 50Hz  (Core 1)
#define TASK_MICROROS_PERIOD_MS 20   // 50Hz  (Core 0) — legacy, giữ để TaskMicroROS compile
#define TASK_SERIALCOMM_PERIOD_MS 20 // 50Hz  (Core 0) — raw serial V2
#define TASK_UART_PERIOD_MS     50   // 20Hz  (Core 0)
#define TASK_LOGGER_PERIOD_MS   200  // 5Hz   (Core 0)

#define TASK_MOTION_PRIORITY    5    // Cao nhat
#define TASK_MICROROS_PRIORITY  3    // legacy
#define TASK_SERIALCOMM_PRIORITY 3   // raw serial V2, cùng priority với micro-ROS
#define TASK_UART_PRIORITY      2
#define TASK_LOGGER_PRIORITY    1    // Thap nhat

#define TASK_MOTION_STACK    4096
#define TASK_MICROROS_STACK  8192    // micro-ROS can nhieu stack — legacy
#define TASK_SERIALCOMM_STACK 4096  // raw serial V2 (không cần 8K như micro-ROS)
#define TASK_UART_STACK      3072
#define TASK_LOGGER_STACK    3072


// ==========================================
// VELOCITY RAMP (Tang/Giam toc mem)
// Bao ve banh rang, chong giat khi bat dau/dung
// ==========================================
#define RAMP_ACCEL_VX 1.5f  // m/s^2
#define RAMP_ACCEL_WZ 60.0f // rad/s^2

// ==========================================
// IMU SENSOR FUSION PARAMETERS (Adaptive Complementary Filter)
// Alpha thay doi theo trang thai chuyen dong:
//   - Xoay (|gyro_z| > threshold): alpha CAO → tin gyro (encoder bi truot)
//   - Thang (|gyro_z| <= threshold): alpha THAP → blend deu gyro + encoder
// ==========================================
#define COMP_ALPHA_HIGH      0.995f  // Khi xoay: tin gyro 99.5%
#define COMP_ALPHA_LOW       0.95f   // Khi di thang: tin gyro 95%, encoder 5%
#define COMP_GYRO_THRESHOLD  0.1f    // Nguong chuyen doi (rad/s)

// ==========================================
// WATCHDOG / SAFETY
// ==========================================
#define CMD_VEL_TIMEOUT_MS 1000  // Mat lenh > 1s -> EMERGENCY

// ==========================================
// DEBUG SERIAL — Tach rieng debug log va serial transport
// ==========================================
// Serial  (USB CDC) → Raw Serial V2 transport (noi voi Raspberry Pi)
// Serial0 (UART0/CH340) → Debug log + Serial Monitor (noi voi Laptop)
//
// Tren board Freenove ESP32-S3 co 2 cong USB Type-C:
//   Cong "UART" (CH340) = Serial0 → dung cho nap code + debug
//   Cong "USB"  (native) = Serial  → dung cho raw serial V2
#define DBG Serial0

#endif // ROBOT_CONFIG_H
