#pragma once
#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ── Shared state (all access guarded by g_state_mutex) ──────────
extern SemaphoreHandle_t g_state_mutex;

// IMU data (written by imuTask, read by microrosTask)
struct ImuData {
    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float accel_z = 0.0f;
    float gyro_x  = 0.0f;
    float gyro_y  = 0.0f;
    float gyro_z  = 0.0f;
    bool  valid   = false;
};

// Magnetometer data (written by imuTask, read by microrosTask)
struct MagData {
    float mag_x   = 0.0f;
    float mag_y   = 0.0f;
    float mag_z   = 0.0f;
    bool  valid   = false;
};

// Mag calibration state (written/read by imuTask, commands from microrosTask)
struct MagCalibrationState {
    bool    active            = false;
    bool    calibration_done  = false;
    bool    has_samples       = false;
    bool    trigger_requested = false;   // set by microrosTask callback, consumed by imuTask
    uint32_t start_ms         = 0;
    float   min_x = 0.0f, max_x = 0.0f;
    float   min_y = 0.0f, max_y = 0.0f;
    float   min_z = 0.0f, max_z = 0.0f;
    float   offset_x = 0.0f, offset_y = 0.0f, offset_z = 0.0f;
};

extern ImuData              g_imu_data;
extern MagData              g_mag_data;
extern MagCalibrationState  g_mag_cal;
extern bool                 g_agent_connected;

// ── FreeRTOS task functions ─────────────────────────────────────
void imuTask(void* pvParams);
