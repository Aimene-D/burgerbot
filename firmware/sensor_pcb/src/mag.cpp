#include "mag.h"
#include "tasks.h"       // for g_mag_cal shared state
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_HMC5883_U.h>
#include <Preferences.h>

// ── Statics ──────────────────────────────────────────────────────
static Adafruit_HMC5883_Unified s_hmc5883(5883);
static bool     s_compass_present   = false;
static bool     s_compass_using_qmc = false;

// Raw (uncalibrated) readings, updated each readCompass()
static float    s_raw_mag_x = 0.0f;
static float    s_raw_mag_y = 0.0f;
static float    s_raw_mag_z = 0.0f;

// Calibrated readings (raw - offset)
static float    s_mag_x = 0.0f;
static float    s_mag_y = 0.0f;
static float    s_mag_z = 0.0f;

// Yaw from complementary filter (gyro + mag)
static float    s_yaw_rad         = 0.0f;
static bool     s_yaw_initialized = false;
static uint32_t s_yaw_last_ms     = 0;

// Stale detection
static uint32_t s_compass_stale_ms = 0;
static float    s_prev_mag_x = 0.0f, s_prev_mag_y = 0.0f, s_prev_mag_z = 0.0f;
static uint32_t s_prev_read_ms = 0;

static Preferences g_mag_prefs;

// ── I2C low-level helpers ────────────────────────────────────────
static bool probeI2cAddress(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

static bool writeI2cRegister(uint8_t address, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool readI2cRegisters(uint8_t address, uint8_t start_reg, uint8_t* out_data, size_t len) {
    Wire.beginTransmission(address);
    Wire.write(start_reg);
    if (Wire.endTransmission(false) != 0) return false;
    const size_t read_count = Wire.requestFrom(static_cast<int>(address), static_cast<int>(len), 1);
    if (read_count != len) return false;
    for (size_t i = 0; i < len; ++i) out_data[i] = static_cast<uint8_t>(Wire.read());
    return true;
}

// ── QMC5883 support ──────────────────────────────────────────────
static bool initQmc5883() {
    const bool ctrl2_ok = writeI2cRegister(QMC5883_I2C_ADDR, QMC_REG_CONTROL_2, 0x00);
    const bool reset_ok = writeI2cRegister(QMC5883_I2C_ADDR, QMC_REG_SET_RESET,  0x01);
    const bool ctrl1_ok = writeI2cRegister(QMC5883_I2C_ADDR, QMC_REG_CONTROL_1,  0x15);
    return ctrl1_ok && ctrl2_ok && reset_ok;
}

static bool readQmc5883MagneticField(float& mag_x_ut, float& mag_y_ut, float& mag_z_ut) {
    uint8_t status = 0;
    if (!readI2cRegisters(QMC5883_I2C_ADDR, QMC_REG_STATUS, &status, 1)) return false;
    if ((status & QMC_STATUS_DATA_READY) == 0) return false;
    uint8_t raw[6] = {0};
    if (!readI2cRegisters(QMC5883_I2C_ADDR, QMC_REG_X_LSB, raw, sizeof(raw))) return false;
    const int16_t raw_x = static_cast<int16_t>((static_cast<uint16_t>(raw[1]) << 8) | raw[0]);
    const int16_t raw_y = static_cast<int16_t>((static_cast<uint16_t>(raw[3]) << 8) | raw[2]);
    const int16_t raw_z = static_cast<int16_t>((static_cast<uint16_t>(raw[5]) << 8) | raw[4]);
    mag_x_ut = static_cast<float>(raw_x) * QMC_LSB_TO_UT;
    mag_y_ut = static_cast<float>(raw_y) * QMC_LSB_TO_UT;
    mag_z_ut = static_cast<float>(raw_z) * QMC_LSB_TO_UT;
    return true;
}

// ── Heading computation ──────────────────────────────────────────
static float computeHeadingDegrees(float mag_x, float mag_y, float mag_z) {
    float heading_rad = 0.0f;
    if (ENABLE_TILT_COMPENSATION) {
        // Use IMU accel for tilt compensation
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        float accel_x = g_imu_data.accel_x;
        float accel_y = g_imu_data.accel_y;
        float accel_z = g_imu_data.accel_z;
        bool imu_valid = g_imu_data.valid;
        xSemaphoreGive(g_state_mutex);

        if (imu_valid) {
            const float roll  = atan2f(accel_y, accel_z);
            const float pitch = atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z));
            const float xh = (mag_x * cosf(pitch)) + (mag_z * sinf(pitch));
            const float yh = (mag_x * sinf(roll) * sinf(pitch)) + (mag_y * cosf(roll)) -
                             (mag_z * sinf(roll) * cosf(pitch));
            heading_rad = atan2f(yh, xh);
        } else {
            heading_rad = atan2f(mag_y, mag_x);
        }
    } else {
        heading_rad = atan2f(mag_y, mag_x);
    }

    heading_rad += COMPASS_DECLINATION_DEG * static_cast<float>(M_PI) / 180.0f;
    while (heading_rad < 0.0f)                            heading_rad += 2.0f * static_cast<float>(M_PI);
    while (heading_rad >= 2.0f * static_cast<float>(M_PI)) heading_rad -= 2.0f * static_cast<float>(M_PI);
    return heading_rad * 180.0f / static_cast<float>(M_PI);
}

// ═════════════════════════════════════════════════════════════════
// Public API
// ═════════════════════════════════════════════════════════════════

bool initMag() {
    const bool compass_addr_ok = probeI2cAddress(HMC5883L_I2C_ADDR);
    const bool qmc_addr_ok     = probeI2cAddress(QMC5883_I2C_ADDR);

    // Try HMC5883L first
    s_compass_present = s_hmc5883.begin();
    s_compass_using_qmc = false;

    // s_hmc5883.begin() calls Wire.begin() internally which resets pins to
    // defaults, so re-init Wire to restore our custom I2C pins
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);

    if (!s_compass_present && qmc_addr_ok) {
        s_compass_present   = initQmc5883();
        s_compass_using_qmc = s_compass_present;
    }

    if (!s_compass_present) {
        ESP_LOGE("MAG", "No magnetometer found");
        return false;
    }

    // Load saved calibration offsets
    loadMagOffsetsFromNvs();

    ESP_LOGI("MAG", "%s found — ready%s",
             s_compass_using_qmc ? "QMC5883" : "HMC5883L",
             g_mag_cal.calibration_done ? " (calibrated)" : " (uncalibrated)");
    return true;
}

void readCompass(uint32_t now_ms) {
    if (!s_compass_present) return;
    static uint32_t last_read_ms = 0;
    if ((now_ms - last_read_ms) < COMPASS_READ_PERIOD_MS) return;
    last_read_ms = now_ms;

    float mag_x = 0.0f, mag_y = 0.0f, mag_z = 0.0f;
    if (s_compass_using_qmc) {
        if (!readQmc5883MagneticField(mag_x, mag_y, mag_z)) return;
    } else {
        sensors_event_t event;
        s_hmc5883.getEvent(&event);
        mag_x = event.magnetic.x;
        mag_y = event.magnetic.y;
        mag_z = event.magnetic.z;
    }

    // ── Calibration sampling ─────────────────────────────────────
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    if (g_mag_cal.active) {
        if (!g_mag_cal.has_samples) {
            g_mag_cal.min_x = g_mag_cal.max_x = mag_x;
            g_mag_cal.min_y = g_mag_cal.max_y = mag_y;
            g_mag_cal.min_z = g_mag_cal.max_z = mag_z;
            g_mag_cal.has_samples = true;
        } else {
            g_mag_cal.min_x = min(g_mag_cal.min_x, mag_x);
            g_mag_cal.max_x = max(g_mag_cal.max_x, mag_x);
            g_mag_cal.min_y = min(g_mag_cal.min_y, mag_y);
            g_mag_cal.max_y = max(g_mag_cal.max_y, mag_y);
            g_mag_cal.min_z = min(g_mag_cal.min_z, mag_z);
            g_mag_cal.max_z = max(g_mag_cal.max_z, mag_z);
        }
    }
    xSemaphoreGive(g_state_mutex);

    // ── Stale detection ──────────────────────────────────────────
    const float dx = fabsf(mag_x - s_prev_mag_x);
    const float dy = fabsf(mag_y - s_prev_mag_y);
    const float dz = fabsf(mag_z - s_prev_mag_z);
    if ((dx < COMPASS_STALE_EPS_UT) && (dy < COMPASS_STALE_EPS_UT) && (dz < COMPASS_STALE_EPS_UT)) {
        if (s_prev_read_ms > 0) s_compass_stale_ms += (now_ms - s_prev_read_ms);
    } else {
        s_compass_stale_ms = 0;
    }
    s_prev_mag_x = mag_x;
    s_prev_mag_y = mag_y;
    s_prev_mag_z = mag_z;
    s_prev_read_ms = now_ms;

    // ── Apply calibration offsets ────────────────────────────────
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    mag_x -= g_mag_cal.offset_x;
    mag_y -= g_mag_cal.offset_y;
    mag_z -= g_mag_cal.offset_z;
    xSemaphoreGive(g_state_mutex);

    if (isnan(mag_x) || isnan(mag_y) || isnan(mag_z) ||
        isinf(mag_x) || isinf(mag_y) || isinf(mag_z)) {
        return;
    }

    s_raw_mag_x = mag_x;
    s_raw_mag_y = mag_y;
    s_raw_mag_z = mag_z;
}

void updateYawEstimate(uint32_t now_ms) {
    if (!s_compass_present) return;

    if (!s_yaw_initialized) {
        // Seed yaw from compass
        s_yaw_rad = atan2f(s_raw_mag_y, s_raw_mag_x);
        s_yaw_last_ms = now_ms;
        s_yaw_initialized = true;
        return;
    }

    if (now_ms <= s_yaw_last_ms) return;
    const float dt_s = static_cast<float>(now_ms - s_yaw_last_ms) / 1000.0f;
    s_yaw_last_ms = now_ms;
    if (dt_s <= 0.0f) return;

    // Gyro prediction
    float gyro_pred_yaw = s_yaw_rad;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    bool imu_valid = g_imu_data.valid;
    float gyro_z = g_imu_data.gyro_z;
    xSemaphoreGive(g_state_mutex);

    if (imu_valid) {
        gyro_pred_yaw = s_yaw_rad + (gyro_z * dt_s);
        // Normalize
        while (gyro_pred_yaw >  static_cast<float>(M_PI)) gyro_pred_yaw -= 2.0f * static_cast<float>(M_PI);
        while (gyro_pred_yaw < -static_cast<float>(M_PI)) gyro_pred_yaw += 2.0f * static_cast<float>(M_PI);
    }

    // Complementary: gyro + magnetometer
    const float compass_yaw = atan2f(s_raw_mag_y, s_raw_mag_x);
    s_yaw_rad = (YAW_COMPLEMENTARY_ALPHA * gyro_pred_yaw) +
                ((1.0f - YAW_COMPLEMENTARY_ALPHA) * compass_yaw);
    // Normalize
    while (s_yaw_rad >  static_cast<float>(M_PI)) s_yaw_rad -= 2.0f * static_cast<float>(M_PI);
    while (s_yaw_rad < -static_cast<float>(M_PI)) s_yaw_rad += 2.0f * static_cast<float>(M_PI);
}

float getMagX() { return s_raw_mag_x; }
float getMagY() { return s_raw_mag_y; }
float getMagZ() { return s_raw_mag_z; }
float getYawRad() { return s_yaw_rad; }

bool isMagStale() {
    return s_compass_stale_ms >= COMPASS_STALE_WARN_MS;
}

// ═════════════════════════════════════════════════════════════════
// Calibration
// ═════════════════════════════════════════════════════════════════

void startMagCalibration(uint32_t now_ms) {
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_mag_cal.active           = true;
    g_mag_cal.calibration_done = false;
    g_mag_cal.has_samples      = false;
    g_mag_cal.start_ms         = now_ms;
    xSemaphoreGive(g_state_mutex);

    ESP_LOGI("MAG", "Calibration started — rotate robot 360°");
}

void finishMagCalibration(bool persist_offsets) {
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    if (!g_mag_cal.active) { xSemaphoreGive(g_state_mutex); return; }
    g_mag_cal.active = false;

    if (!g_mag_cal.has_samples) {
        g_mag_cal.calibration_done = false;
        xSemaphoreGive(g_state_mutex);
        ESP_LOGW("MAG", "Calibration finished with no samples");
        return;
    }

    g_mag_cal.offset_x = 0.5f * (g_mag_cal.max_x + g_mag_cal.min_x);
    g_mag_cal.offset_y = 0.5f * (g_mag_cal.max_y + g_mag_cal.min_y);
    g_mag_cal.offset_z = 0.5f * (g_mag_cal.max_z + g_mag_cal.min_z);
    g_mag_cal.calibration_done = true;

    const float off_x = g_mag_cal.offset_x;
    const float off_y = g_mag_cal.offset_y;
    const float off_z = g_mag_cal.offset_z;
    xSemaphoreGive(g_state_mutex);

    if (persist_offsets) {
        saveMagOffsetsToNvs(off_x, off_y, off_z);
    }

    ESP_LOGI("MAG", "Calibration done: off_x=%.3f off_y=%.3f off_z=%.3f",
             off_x, off_y, off_z);
}

void updateMagCalibration(uint32_t now_ms) {
    // Check if calibration duration exceeded
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    const bool active = g_mag_cal.active;
    const uint32_t start_ms = g_mag_cal.start_ms;
    xSemaphoreGive(g_state_mutex);

    if (!active) return;

    if ((now_ms - start_ms) >= MAG_CALIBRATION_DURATION_MS) {
        finishMagCalibration(true);
    }
}

void saveMagOffsetsToNvs(float off_x, float off_y, float off_z) {
    if (!g_mag_prefs.begin(MAG_PREFS_NAMESPACE, false)) return;
    g_mag_prefs.putFloat(MAG_PREF_KEY_OFFSET_X, off_x);
    g_mag_prefs.putFloat(MAG_PREF_KEY_OFFSET_Y, off_y);
    g_mag_prefs.putFloat(MAG_PREF_KEY_OFFSET_Z, off_z);
    g_mag_prefs.end();
    ESP_LOGI("MAG", "Offsets saved to NVS");
}

void loadMagOffsetsFromNvs() {
    if (!g_mag_prefs.begin(MAG_PREFS_NAMESPACE, true)) {
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        g_mag_cal.offset_x = HMC_OFFSET_X_UT;
        g_mag_cal.offset_y = HMC_OFFSET_Y_UT;
        g_mag_cal.offset_z = HMC_OFFSET_Z_UT;
        g_mag_cal.calibration_done = false;
        xSemaphoreGive(g_state_mutex);
        return;
    }

    const bool has_x = g_mag_prefs.isKey(MAG_PREF_KEY_OFFSET_X);
    const bool has_y = g_mag_prefs.isKey(MAG_PREF_KEY_OFFSET_Y);
    const bool has_z = g_mag_prefs.isKey(MAG_PREF_KEY_OFFSET_Z);

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_mag_cal.offset_x = has_x ? g_mag_prefs.getFloat(MAG_PREF_KEY_OFFSET_X, HMC_OFFSET_X_UT) : HMC_OFFSET_X_UT;
    g_mag_cal.offset_y = has_y ? g_mag_prefs.getFloat(MAG_PREF_KEY_OFFSET_Y, HMC_OFFSET_Y_UT) : HMC_OFFSET_Y_UT;
    g_mag_cal.offset_z = has_z ? g_mag_prefs.getFloat(MAG_PREF_KEY_OFFSET_Z, HMC_OFFSET_Z_UT) : HMC_OFFSET_Z_UT;
    g_mag_cal.calibration_done = has_x && has_y && has_z;
    xSemaphoreGive(g_state_mutex);

    g_mag_prefs.end();
}
