#pragma once
#include <Arduino.h>
#include "config.h"

struct ImuData {
  float accel_x = 0.0f, accel_y = 0.0f, accel_z = 0.0f;
  float gyro_x  = 0.0f, gyro_y  = 0.0f, gyro_z  = 0.0f;
  uint32_t last_read_ms = 0;
};

struct CompassData {
  float mag_x = 0.0f, mag_y = 0.0f, mag_z = 0.0f;
  float yaw_rad = 0.0f, heading_deg = 0.0f;
  uint32_t last_read_ms = 0;
};

struct OrientationState {
  float yaw_rad = 0.0f;
  bool initialized = false;
  uint32_t last_update_ms = 0;
};

struct MagCalibrationState {
  bool active = false, calibration_done = false;
  bool has_samples = false, auto_rotate = true;
  uint32_t start_ms = 0;
  uint32_t duration_ms = MAG_CALIBRATION_DURATION_MS;
  float angular_speed_radps = MAG_CALIBRATION_ANGULAR_RADPS;
  float min_x = 0.0f, max_x = 0.0f;
  float min_y = 0.0f, max_y = 0.0f;
  float min_z = 0.0f, max_z = 0.0f;
  float offset_x = 0.0f, offset_y = 0.0f, offset_z = 0.0f;
};

extern ImuData          g_imu_data;
extern CompassData      g_compass_data;
extern OrientationState g_orientation;
extern MagCalibrationState g_mag_calibration;
extern bool g_imu_data_valid;
extern bool g_compass_data_valid;

void initI2cAndSensors();
void readImuData(uint32_t now_ms);
void readCompassData(uint32_t now_ms);
void updateYawEstimate(uint32_t now_ms);
void logSensorData(uint32_t now_ms);
void startMagCalibration(uint32_t now_ms);
void finishMagCalibration(bool persist_offsets);
void updateMagCalibrationMode(uint32_t now_ms);
void loadMagOffsetsFromNvs();
float normalizeAngleRad(float angle_rad);
