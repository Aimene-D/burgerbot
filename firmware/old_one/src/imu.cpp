#include "imu.h"
#include "motors.h"
#include "pid.h"
#include <Wire.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_LSM6DS3.h>
#include <Preferences.h>

// ─── Globals ──────────────────────────────────────────────
ImuData            g_imu_data;
CompassData        g_compass_data;
OrientationState   g_orientation;
MagCalibrationState g_mag_calibration;
bool g_imu_data_valid    = false;
bool g_compass_data_valid = false;

static Adafruit_LSM6DS3       g_lsm6ds3;
static Adafruit_HMC5883_Unified g_hmc5883(5883);
static uint8_t  g_lsm6ds3_i2c_addr  = LSM6DS3_I2C_ADDR_PRIMARY;
static bool     g_imu_present        = false;
static bool     g_compass_present    = false;
static bool     g_compass_using_qmc  = false;
static uint32_t g_compass_stale_ms   = 0;
static Preferences g_mag_prefs;

// ─── Helpers ──────────────────────────────────────────────
float normalizeAngleRad(float angle_rad) {
  while (angle_rad >  static_cast<float>(M_PI)) angle_rad -= 2.0f * static_cast<float>(M_PI);
  while (angle_rad < -static_cast<float>(M_PI)) angle_rad += 2.0f * static_cast<float>(M_PI);
  return angle_rad;
}

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

static float computeHeadingDegrees(float mag_x, float mag_y, float mag_z) {
  float heading_rad = 0.0f;
  if (ENABLE_TILT_COMPENSATION && g_imu_data_valid) {
    const float roll  = atan2f(g_imu_data.accel_y, g_imu_data.accel_z);
    const float pitch = atan2f(-g_imu_data.accel_x,
                               sqrtf((g_imu_data.accel_y * g_imu_data.accel_y) +
                                     (g_imu_data.accel_z * g_imu_data.accel_z)));
    const float xh = (mag_x * cosf(pitch)) + (mag_z * sinf(pitch));
    const float yh = (mag_x * sinf(roll) * sinf(pitch)) + (mag_y * cosf(roll)) -
                     (mag_z * sinf(roll) * cosf(pitch));
    heading_rad = atan2f(yh, xh);
  } else {
    heading_rad = atan2f(mag_y, mag_x);
  }
  heading_rad += COMPASS_DECLINATION_DEG * static_cast<float>(M_PI) / 180.0f;
  while (heading_rad < 0.0f)                          heading_rad += 2.0f * static_cast<float>(M_PI);
  while (heading_rad >= 2.0f * static_cast<float>(M_PI)) heading_rad -= 2.0f * static_cast<float>(M_PI);
  return heading_rad * 180.0f / static_cast<float>(M_PI);
}

// ─── Public functions ──────────────────────────────────────
void initI2cAndSensors() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  delay(80);

  const bool imu_primary_ok  = probeI2cAddress(LSM6DS3_I2C_ADDR_PRIMARY);
  const bool imu_alt_ok      = probeI2cAddress(LSM6DS3_I2C_ADDR_ALT);
  const bool compass_addr_ok = probeI2cAddress(HMC5883L_I2C_ADDR);
  const bool qmc_addr_ok     = probeI2cAddress(QMC5883_I2C_ADDR);

  if (imu_primary_ok)      g_lsm6ds3_i2c_addr = LSM6DS3_I2C_ADDR_PRIMARY;
  else if (imu_alt_ok)     g_lsm6ds3_i2c_addr = LSM6DS3_I2C_ADDR_ALT;

  g_imu_present     = g_lsm6ds3.begin_I2C(g_lsm6ds3_i2c_addr, &Wire);
  g_compass_present = g_hmc5883.begin();
  g_compass_using_qmc = false;
  if (!g_compass_present && qmc_addr_ok) {
    g_compass_present   = initQmc5883();
    g_compass_using_qmc = g_compass_present;
  }

  Serial.printf("I2C SDA=%d SCL=%d Freq=%luHz\n", I2C_SDA_PIN, I2C_SCL_PIN,
                static_cast<unsigned long>(I2C_FREQ_HZ));
  Serial.printf("I2C probe: LSM6DS3(0x6A)=%s LSM6DS3(0x6B)=%s HMC5883L(0x1E)=%s\n",
                imu_primary_ok ? "OK" : "MISS", imu_alt_ok ? "OK" : "MISS",
                compass_addr_ok ? "OK" : "MISS");
  if (!compass_addr_ok && qmc_addr_ok)
    Serial.println("WARNING: device found at 0x0D (QMC5883).");
  Serial.printf("Sensor init: IMU=%s addr=0x%02X COMPASS=%s (%s)\n",
                g_imu_present ? "OK" : "FAILED", g_lsm6ds3_i2c_addr,
                g_compass_present ? "OK" : "FAILED",
                g_compass_using_qmc ? "QMC5883" : "HMC5883L");
}

void readImuData(uint32_t now_ms) {
  if (!g_imu_present || ((now_ms - g_imu_data.last_read_ms) < IMU_READ_PERIOD_MS)) return;
  sensors_event_t accel, gyro, temp;
  if (g_lsm6ds3.getEvent(&accel, &gyro, &temp)) {
    g_imu_data.accel_x = accel.acceleration.x;
    g_imu_data.accel_y = accel.acceleration.y;
    g_imu_data.accel_z = accel.acceleration.z;
    g_imu_data.gyro_x  = gyro.gyro.x - 0.0586f;
    g_imu_data.gyro_y  = gyro.gyro.y + 0.0880f;
    g_imu_data.gyro_z  = gyro.gyro.z + 0.0910f;
    g_imu_data.last_read_ms = now_ms;
    g_imu_data_valid = true;
  } else {
    g_imu_data_valid = false;
  }
}

void readCompassData(uint32_t now_ms) {
  if (!g_compass_present || ((now_ms - g_compass_data.last_read_ms) < COMPASS_READ_PERIOD_MS)) return;
  const float prev_x = g_compass_data.mag_x;
  const float prev_y = g_compass_data.mag_y;
  const float prev_z = g_compass_data.mag_z;
  const uint32_t prev_read_ms = g_compass_data.last_read_ms;
  float mag_x = 0.0f, mag_y = 0.0f, mag_z = 0.0f;
  if (g_compass_using_qmc) {
    if (!readQmc5883MagneticField(mag_x, mag_y, mag_z)) return;
  } else {
    sensors_event_t event;
    g_hmc5883.getEvent(&event);
    mag_x = event.magnetic.x;
    mag_y = event.magnetic.y;
    mag_z = event.magnetic.z;
  }
  if (g_mag_calibration.active) {
    if (!g_mag_calibration.has_samples) {
      g_mag_calibration.min_x = g_mag_calibration.max_x = mag_x;
      g_mag_calibration.min_y = g_mag_calibration.max_y = mag_y;
      g_mag_calibration.min_z = g_mag_calibration.max_z = mag_z;
      g_mag_calibration.has_samples = true;
    } else {
      g_mag_calibration.min_x = min(g_mag_calibration.min_x, mag_x);
      g_mag_calibration.max_x = max(g_mag_calibration.max_x, mag_x);
      g_mag_calibration.min_y = min(g_mag_calibration.min_y, mag_y);
      g_mag_calibration.max_y = max(g_mag_calibration.max_y, mag_y);
      g_mag_calibration.min_z = min(g_mag_calibration.min_z, mag_z);
      g_mag_calibration.max_z = max(g_mag_calibration.max_z, mag_z);
    }
  }
  mag_x -= g_mag_calibration.offset_x;
  mag_y -= g_mag_calibration.offset_y;
  mag_z -= g_mag_calibration.offset_z;
  if (isnan(mag_x) || isnan(mag_y) || isnan(mag_z) ||
      isinf(mag_x) || isinf(mag_y) || isinf(mag_z)) {
    g_compass_data_valid = false;
    return;
  }
  g_compass_data.mag_x       = mag_x;
  g_compass_data.mag_y       = mag_y;
  g_compass_data.mag_z       = mag_z;
  g_compass_data.yaw_rad     = normalizeAngleRad(atan2f(mag_y, mag_x));
  g_compass_data.heading_deg = computeHeadingDegrees(mag_x, mag_y, mag_z);
  g_compass_data.last_read_ms = now_ms;
  g_compass_data_valid = true;
  const float dx = fabsf(mag_x - prev_x);
  const float dy = fabsf(mag_y - prev_y);
  const float dz = fabsf(mag_z - prev_z);
  if ((dx < COMPASS_STALE_EPS_UT) && (dy < COMPASS_STALE_EPS_UT) && (dz < COMPASS_STALE_EPS_UT)) {
    if (prev_read_ms > 0) g_compass_stale_ms += (now_ms - prev_read_ms);
  } else {
    g_compass_stale_ms = 0;
  }
}

void updateYawEstimate(uint32_t now_ms) {
  if (!g_imu_data_valid && !g_compass_data_valid) return;
  if (!g_orientation.initialized) {
    if (g_compass_data_valid) g_orientation.yaw_rad = g_compass_data.yaw_rad;
    g_orientation.last_update_ms = now_ms;
    g_orientation.initialized = true;
    return;
  }
  if (now_ms <= g_orientation.last_update_ms) return;
  const float dt_s = static_cast<float>(now_ms - g_orientation.last_update_ms) / 1000.0f;
  g_orientation.last_update_ms = now_ms;
  if (dt_s <= 0.0f) return;
  float gyro_pred_yaw = g_orientation.yaw_rad;
  if (g_imu_data_valid)
    gyro_pred_yaw = normalizeAngleRad(g_orientation.yaw_rad + (g_imu_data.gyro_z * dt_s));
  if (g_compass_data_valid) {
    g_orientation.yaw_rad = normalizeAngleRad(
        (YAW_COMPLEMENTARY_ALPHA * gyro_pred_yaw) +
        ((1.0f - YAW_COMPLEMENTARY_ALPHA) * g_compass_data.yaw_rad));
  } else {
    g_orientation.yaw_rad = gyro_pred_yaw;
  }
}

void logSensorData(uint32_t now_ms) {
  static uint32_t last_sensor_log_ms = 0;
  if ((now_ms - last_sensor_log_ms) < SENSOR_DEBUG_PERIOD_MS) return;
  last_sensor_log_ms = now_ms;
  Serial.printf("IMU[%s] A[%.2f %.2f %.2f] G[%.3f %.3f %.3f] | MAG[%s] M[%.2f %.2f %.2f] HDG=%.1f\n",
                g_imu_data_valid ? "OK" : "NA",
                g_imu_data.accel_x, g_imu_data.accel_y, g_imu_data.accel_z,
                g_imu_data.gyro_x,  g_imu_data.gyro_y,  g_imu_data.gyro_z,
                g_compass_data_valid ? "OK" : "NA",
                g_compass_data.mag_x, g_compass_data.mag_y, g_compass_data.mag_z,
                g_compass_data.heading_deg);
  if (g_compass_stale_ms >= COMPASS_STALE_WARN_MS)
    Serial.printf("WARNING: compass unchanged for %lums.\n",
                  static_cast<unsigned long>(g_compass_stale_ms));
}

void startMagCalibration(uint32_t now_ms) {
  g_mag_calibration.active           = true;
  g_mag_calibration.calibration_done = false;
  g_mag_calibration.has_samples      = false;
  g_mag_calibration.start_ms         = now_ms;
  setTargetsFromCmdVel(0.0f, g_mag_calibration.angular_speed_radps);
  g_last_cmd_ms = now_ms;
  Serial.printf("Mag calibration started (duration=%lums, omega=%.2f rad/s)\n",
                static_cast<unsigned long>(g_mag_calibration.duration_ms),
                g_mag_calibration.angular_speed_radps);
}

void finishMagCalibration(bool persist_offsets) {
  if (!g_mag_calibration.active) return;
  g_mag_calibration.active = false;
  g_m1_target_rpm = 0.0f;
  g_m2_target_rpm = 0.0f;
  g_last_cmd_ms   = millis();
  if (!g_mag_calibration.has_samples) {
    g_mag_calibration.calibration_done = false;
    Serial.println("Mag calibration finished without samples.");
    return;
  }
  g_mag_calibration.offset_x = 0.5f * (g_mag_calibration.max_x + g_mag_calibration.min_x);
  g_mag_calibration.offset_y = 0.5f * (g_mag_calibration.max_y + g_mag_calibration.min_y);
  g_mag_calibration.offset_z = 0.5f * (g_mag_calibration.max_z + g_mag_calibration.min_z);
  g_mag_calibration.calibration_done = true;
  if (persist_offsets)
    saveMagOffsetsToNvs(g_mag_calibration.offset_x,
                        g_mag_calibration.offset_y,
                        g_mag_calibration.offset_z);
  Serial.printf("Mag calibration done: off_x=%.3f off_y=%.3f off_z=%.3f\n",
                g_mag_calibration.offset_x,
                g_mag_calibration.offset_y,
                g_mag_calibration.offset_z);
}

void updateMagCalibrationMode(uint32_t now_ms) {
  if (!g_mag_calibration.active || !g_mag_calibration.auto_rotate) return;
  setTargetsFromCmdVel(0.0f, g_mag_calibration.angular_speed_radps);
  g_last_cmd_ms = now_ms;
  if ((now_ms - g_mag_calibration.start_ms) >= g_mag_calibration.duration_ms)
    finishMagCalibration(true);
}

void loadMagOffsetsFromNvs() {
  if (!g_mag_prefs.begin(MAG_PREFS_NAMESPACE, true)) {
    g_mag_calibration.offset_x         = HMC_OFFSET_X_UT;
    g_mag_calibration.offset_y         = HMC_OFFSET_Y_UT;
    g_mag_calibration.offset_z         = HMC_OFFSET_Z_UT;
    g_mag_calibration.calibration_done = false;
    return;
  }
  const bool has_x = g_mag_prefs.isKey(MAG_PREF_KEY_OFFSET_X);
  const bool has_y = g_mag_prefs.isKey(MAG_PREF_KEY_OFFSET_Y);
  const bool has_z = g_mag_prefs.isKey(MAG_PREF_KEY_OFFSET_Z);
  g_mag_calibration.offset_x = has_x ? g_mag_prefs.getFloat(MAG_PREF_KEY_OFFSET_X, HMC_OFFSET_X_UT) : HMC_OFFSET_X_UT;
  g_mag_calibration.offset_y = has_y ? g_mag_prefs.getFloat(MAG_PREF_KEY_OFFSET_Y, HMC_OFFSET_Y_UT) : HMC_OFFSET_Y_UT;
  g_mag_calibration.offset_z = has_z ? g_mag_prefs.getFloat(MAG_PREF_KEY_OFFSET_Z, HMC_OFFSET_Z_UT) : HMC_OFFSET_Z_UT;
  g_mag_calibration.calibration_done = has_x && has_y && has_z;
  g_mag_prefs.end();
}
