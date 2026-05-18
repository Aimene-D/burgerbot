#pragma once

// ─── Motor pins ───────────────────────────────────────────
constexpr int M1_RPWM_PIN = 17;
constexpr int M1_LPWM_PIN = 16;
constexpr int M1_REN_PIN  = 15;
constexpr int M1_LEN_PIN  = 7;

constexpr int M2_RPWM_PIN = 47;
constexpr int M2_LPWM_PIN = 21;
constexpr int M2_REN_PIN  = 2;
constexpr int M2_LEN_PIN  = 1;

// ─── Encoder pins ─────────────────────────────────────────
constexpr int M1_ENC_B_PIN = 5;
constexpr int M1_ENC_A_PIN = 6;
constexpr int M2_ENC_B_PIN = 38;
constexpr int M2_ENC_A_PIN = 37;

// ─── I2C ──────────────────────────────────────────────────
constexpr int      I2C_SDA_PIN              = 8;
constexpr int      I2C_SCL_PIN              = 9;
constexpr uint32_t I2C_FREQ_HZ              = 400000;
constexpr uint8_t  LSM6DS3_I2C_ADDR_PRIMARY = 0x6A;
constexpr uint8_t  LSM6DS3_I2C_ADDR_ALT     = 0x6B;
constexpr uint8_t  HMC5883L_I2C_ADDR        = 0x1E;
constexpr uint8_t  QMC5883_I2C_ADDR         = 0x0D;
constexpr uint8_t  QMC_REG_X_LSB            = 0x00;
constexpr uint8_t  QMC_REG_STATUS           = 0x06;
constexpr uint8_t  QMC_REG_CONTROL_1        = 0x09;
constexpr uint8_t  QMC_REG_CONTROL_2        = 0x0A;
constexpr uint8_t  QMC_REG_SET_RESET        = 0x0B;
constexpr uint8_t  QMC_STATUS_DATA_READY    = 0x01;
constexpr float    QMC_LSB_TO_UT            = 0.1f;

// ─── IMU / Compass timing ─────────────────────────────────
constexpr uint32_t IMU_READ_PERIOD_MS        = 20;
constexpr uint32_t COMPASS_READ_PERIOD_MS    = 50;
constexpr uint32_t SENSOR_DEBUG_PERIOD_MS    = 1000;
constexpr float    YAW_COMPLEMENTARY_ALPHA   = 0.98f;
constexpr float    COMPASS_DECLINATION_DEG   = 0.0f;
constexpr float    HMC_OFFSET_X_UT          = 0.0f;
constexpr float    HMC_OFFSET_Y_UT          = 0.0f;
constexpr float    HMC_OFFSET_Z_UT          = 0.0f;
constexpr bool     ENABLE_TILT_COMPENSATION  = false;
constexpr float    COMPASS_STALE_EPS_UT      = 0.02f;
constexpr uint32_t COMPASS_STALE_WARN_MS     = 2000;
constexpr uint32_t MAG_CALIBRATION_DURATION_MS   = 15000;
constexpr float    MAG_CALIBRATION_ANGULAR_RADPS = 1.0f;

// ─── PWM ──────────────────────────────────────────────────
constexpr int PWM_FREQ_HZ  = 20000;
constexpr int PWM_RES_BITS = 8;
constexpr int PWM_MAX      = (1 << PWM_RES_BITS) - 1;
constexpr int PWM_M1_R_CH  = 0;
constexpr int PWM_M1_L_CH  = 1;
constexpr int PWM_M2_R_CH  = 2;
constexpr int PWM_M2_L_CH  = 3;

// ─── Kinematics ───────────────────────────────────────────
constexpr float ENCODER_PPR             = 11.0f;
constexpr float GEAR_RATIO              = 18.8f;
constexpr float ENCODER_EDGE_MULTIPLIER = 1.0f;
constexpr float COUNTS_PER_OUTPUT_REV   = ENCODER_PPR * GEAR_RATIO * ENCODER_EDGE_MULTIPLIER;
constexpr float WHEEL_RADIUS_M          = 0.065f;
constexpr float WHEEL_BASE_M            = 0.24f;

// ─── Timing ───────────────────────────────────────────────
constexpr uint32_t CONTROL_PERIOD_MS        = 10;
constexpr uint32_t TELEMETRY_PERIOD_MS      = 50;
constexpr uint32_t ROS_TIME_SYNC_PERIOD_MS  = 5000;
constexpr uint32_t CMD_TIMEOUT_MS           = 300;
constexpr uint32_t RPM_ESTIMATION_WINDOW_MS = 50;
constexpr bool     PUBLISH_RAW_ODOM_TF      = false;

// ─── PID defaults ─────────────────────────────────────────
constexpr float M1_KP = 1.00f;
constexpr float M1_KI = 0.50f;
constexpr float M1_KD = 0.01f;
constexpr float M2_KP = 1.00f;
constexpr float M2_KI = 0.50f;
constexpr float M2_KD = 0.01f;

// ─── Control tuning ───────────────────────────────────────
constexpr float SPEED_LPF_ALPHA    = 0.30f;
constexpr int   PWM_DEADBAND       = 8;
constexpr float ZERO_CMD_RPM_EPS   = 0.8f;
constexpr float ZERO_CMD_MPS_EPS   = 0.01f;
constexpr float ZERO_CMD_RADPS_EPS = 0.05f;
constexpr float RPM_NOISE_EPS      = 0.05f;
constexpr float MAX_PLAUSIBLE_RPM  = 700.0f;

// ─── NVS keys ─────────────────────────────────────────────
constexpr const char* PID_PREFS_NAMESPACE  = "pid";
constexpr const char* PID_PREF_KEY_KP     = "kp";
constexpr const char* PID_PREF_KEY_KI     = "ki";
constexpr const char* PID_PREF_KEY_KD     = "kd";
constexpr const char* MAG_PREFS_NAMESPACE  = "mag";
constexpr const char* MAG_PREF_KEY_OFFSET_X = "offx";
constexpr const char* MAG_PREF_KEY_OFFSET_Y = "offy";
constexpr const char* MAG_PREF_KEY_OFFSET_Z = "offz";

// ─── Units ────────────────────────────────────────────────
constexpr float UT_TO_TESLA = 1.0e-6f;

// ─── Motor direction ──────────────────────────────────────
#ifndef M1_MOTOR_DIR_INVERT
#define M1_MOTOR_DIR_INVERT 1
#endif
#ifndef M2_MOTOR_DIR_INVERT
#define M2_MOTOR_DIR_INVERT 0
#endif
constexpr bool M1_MOTOR_DIR_INVERTED = (M1_MOTOR_DIR_INVERT != 0);
constexpr bool M2_MOTOR_DIR_INVERTED = (M2_MOTOR_DIR_INVERT != 0);
constexpr bool M1_ENCODER_INVERT     = false;
constexpr bool M2_ENCODER_INVERT     = true;

// ─── LiDAR LDS02RR ────────────────────────────────────────
constexpr int      LIDAR_MOTOR_PIN = 19;   // Gate IRLZ44N
constexpr int      LIDAR_RX_PIN    = 20;   // TX LiDAR → RX ESP32
constexpr uint32_t LIDAR_BAUD      = 115200;
