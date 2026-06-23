#pragma once
#include <cstdint>

// LSM6DS3 IMU — 6-axis accel + gyro via I2C
bool initImu();                                           // probe + init LSM6DS3
bool readImu(float& ax, float& ay, float& az,             // accel in m/s²
             float& gx, float& gy, float& gz);            // gyro in rad/s (bias-corrected)

// Gyro bias calibration: collect samples at startup, robot must be still
void calibrateGyroBias();                                 // runs ~5 s at 100 Hz
void getGyroBias(float& bx, float& by, float& bz);        // current bias values (rad/s)
