#pragma once

#include <cstdint>
#include <cmath>

// ═══════════════════════════════════════════════════════════════════════
// GPIO pins — Sensor PCB (CARTE CAPTEUR)
// ═══════════════════════════════════════════════════════════════════════

// WS2812B RGB status LED (same as motor_pcb)
constexpr int STATUS_LED_PIN   = 15;
constexpr int STATUS_LED_COUNT = 1;

// I2C bus
constexpr int      I2C_SDA_PIN              = 8;
constexpr int      I2C_SCL_PIN              = 9;
constexpr uint32_t I2C_FREQ_HZ              = 400000;

// ═══════════════════════════════════════════════════════════════════════
// IMU — LSM6DS3 (accel + gyro)
// ═══════════════════════════════════════════════════════════════════════

constexpr uint8_t  LSM6DS3_I2C_ADDR_PRIMARY = 0x6A;
constexpr uint8_t  LSM6DS3_I2C_ADDR_ALT     = 0x6B;

// ═══════════════════════════════════════════════════════════════════════
// Magnetometer — HMC5883L or QMC5883
// ═══════════════════════════════════════════════════════════════════════

constexpr uint8_t  HMC5883L_I2C_ADDR        = 0x1E;
constexpr uint8_t  QMC5883_I2C_ADDR         = 0x0D;
constexpr uint8_t  QMC_REG_X_LSB            = 0x00;
constexpr uint8_t  QMC_REG_STATUS           = 0x06;
constexpr uint8_t  QMC_REG_CONTROL_1        = 0x09;
constexpr uint8_t  QMC_REG_CONTROL_2        = 0x0A;
constexpr uint8_t  QMC_REG_SET_RESET        = 0x0B;
constexpr uint8_t  QMC_STATUS_DATA_READY    = 0x01;
constexpr float    QMC_LSB_TO_UT            = 0.1f;

// ═══════════════════════════════════════════════════════════════════════
// Timing
// ═══════════════════════════════════════════════════════════════════════

constexpr uint32_t IMU_READ_PERIOD_MS         = 10;      // 100 Hz IMU read
constexpr uint32_t IMU_PUB_DIVIDER            = 1;       // publish IMU every tick (100 Hz)
constexpr uint32_t MAG_PUB_DIVIDER            = 2;       // publish mag every 2nd tick (50 Hz)
constexpr uint32_t COMPASS_READ_PERIOD_MS     = 50;      // 20 Hz compass read
constexpr uint32_t ROS_TIME_SYNC_PERIOD_MS    = 5000;    // 5 s clock sync
constexpr uint32_t SENSOR_DEBUG_PERIOD_MS     = 1000;    // 1 Hz serial debug
constexpr uint32_t WATCHDOG_TIMEOUT_S         = 3;       // TWDT seconds

// ═══════════════════════════════════════════════════════════════════════
// Gyro bias calibration (at startup, robot must be still)
// ═══════════════════════════════════════════════════════════════════════

constexpr uint32_t GYRO_BIAS_SAMPLES         = 500;     // collect at 100 Hz → ~5 s
constexpr float    GYRO_BIAS_SAMPLE_DELAY_MS = 10.0f;   // 10 ms between samples

// ═══════════════════════════════════════════════════════════════════════
// Yaw estimation (complementary filter, internal use only)
// ═══════════════════════════════════════════════════════════════════════

constexpr float  YAW_COMPLEMENTARY_ALPHA     = 0.98f;    // gyro weight
constexpr float  COMPASS_DECLINATION_DEG     = 0.0f;     // local magnetic declination
constexpr bool   ENABLE_TILT_COMPENSATION    = false;    // use accel for tilt comp
constexpr float  COMPASS_STALE_EPS_UT        = 0.02f;    // mag stale detection
constexpr uint32_t COMPASS_STALE_WARN_MS     = 2000;

// ═══════════════════════════════════════════════════════════════════════
// Magnetometer calibration
// ═══════════════════════════════════════════════════════════════════════

constexpr uint32_t MAG_CALIBRATION_DURATION_MS   = 15000;   // 15 s
constexpr float    MAG_CALIBRATION_ANGULAR_RADPS = 1.0f;    // ~57 deg/s
constexpr float    HMC_OFFSET_X_UT              = 0.0f;    // default hard-iron offset
constexpr float    HMC_OFFSET_Y_UT              = 0.0f;
constexpr float    HMC_OFFSET_Z_UT              = 0.0f;

// ═══════════════════════════════════════════════════════════════════════
// micro-ROS
// ═══════════════════════════════════════════════════════════════════════

constexpr int  MICROROS_RETRY_ATTEMPTS       = 30;    // 30 × 200 ms = 6 s
constexpr int  MICROROS_RETRY_DELAY_MS        = 200;
constexpr int  MICROROS_RECONNECT_BACKOFF_MS  = 2000;  // 2 s between full retry cycles

// ═══════════════════════════════════════════════════════════════════════
// NVS keys — magnetometer calibration offsets
// ═══════════════════════════════════════════════════════════════════════

constexpr const char* MAG_PREFS_NAMESPACE       = "mag";
constexpr const char* MAG_PREF_KEY_OFFSET_X     = "offx";
constexpr const char* MAG_PREF_KEY_OFFSET_Y     = "offy";
constexpr const char* MAG_PREF_KEY_OFFSET_Z     = "offz";

// ═══════════════════════════════════════════════════════════════════════
// Units
// ═══════════════════════════════════════════════════════════════════════

constexpr float UT_TO_TESLA = 1.0e-6f;
