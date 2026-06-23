#pragma once
#include <cstdint>
#include "config.h"

// Magnetometer — HMC5883L or QMC5883 with complementary yaw filter + calibration

bool   initMag();                            // probe + init magnetometer
void   readCompass(uint32_t now_ms);         // read mag field (called from imuTask)
void   updateYawEstimate(uint32_t now_ms);   // complementary filter gyro→mag yaw

float  getMagX();                            // latest calibrated mag X (µT)
float  getMagY();                            // latest calibrated mag Y (µT)
float  getMagZ();                            // latest calibrated mag Z (µT)
float  getYawRad();                          // latest complementary yaw estimate (rad)
bool   isMagStale();                         // true if compass data unchanged for too long

// Calibration
void   startMagCalibration(uint32_t now_ms);
void   finishMagCalibration(bool persist_offsets);
void   updateMagCalibration(uint32_t now_ms);

// NVS persistence
void   saveMagOffsetsToNvs(float off_x, float off_y, float off_z);
void   loadMagOffsetsFromNvs();
