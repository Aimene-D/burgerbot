#include "tasks.h"
#include "imu.h"
#include "mag.h"
#include "led.h"
#include "status_codes.h"
#include <Arduino.h>
#include <esp_task_wdt.h>

// ── Shared state definitions ────────────────────────────────────
ImuData             g_imu_data;
MagData             g_mag_data;
MagCalibrationState g_mag_cal;
bool                g_agent_connected = false;

// Forward declarations of sensor init functions
extern bool initImu();
extern bool initMag();

// ═══════════════════════════════════════════════════════════════
// imuTask — Core 0, priority 5, 100 Hz sensor read loop
// ═══════════════════════════════════════════════════════════════
void imuTask(void* pvParams) {
    (void)pvParams;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(IMU_READ_PERIOD_MS);  // 10 ms = 100 Hz

    // ── Phase 1: Sensor init ───────────────────────────────────
    const bool imu_ok = initImu();
    const bool mag_ok = initMag();

    // Subscribe to TWDT AFTER one-time boot init (sensor init + gyro bias
    // calibration take ~6 s and would exceed the default 3 s TWDT timeout)
    esp_task_wdt_add(NULL);

    if (!imu_ok) {
        ESP_LOGE("IMU", "LSM6DS3 init failed — IMU data will be invalid");
    }
    if (!mag_ok) {
        ESP_LOGE("MAG", "Magnetometer init failed — compass data will be invalid");
    }

    ESP_LOGI("IMU", "Sensor init: IMU=%s MAG=%s",
             imu_ok ? "OK" : "FAILED",
             mag_ok ? "OK" : "FAILED");

    // ── Phase 2: Gyro bias calibration (robot must be still) ──
    if (imu_ok) {
        setStatusLedImmediate(LedCode::MAG_CALIB);  // purple during calibration
        calibrateGyroBias();
    }
    setStatusLedImmediate(LedCode::BOOTING);

    // ── Phase 3: Main loop (100 Hz) ──────────────────────────
    uint32_t tick_count = 0;

    while (true) {
        vTaskDelayUntil(&last_wake, period);  // 10 ms tick
        tick_count++;

        uint32_t now = millis();

        // ── 1. Check for mag calibration trigger ─────────────────
        bool cal_triggered = false;
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        if (g_mag_cal.trigger_requested) {
            g_mag_cal.trigger_requested = false;
            cal_triggered = true;
        }
        xSemaphoreGive(g_state_mutex);

        if (cal_triggered) {
            startMagCalibration(now);
        }

        // ── 2. Handle active mag calibration ─────────────────────
        if (g_mag_cal.active) {
            updateMagCalibration(now);
            readCompass(now);
            updateYawEstimate(now);
            esp_task_wdt_reset();
            continue;
        }

        // ── 3. Read IMU (100 Hz) ────────────────────────────────
        bool imu_valid = false;
        float ax = 0.0f, ay = 0.0f, az = 0.0f;
        float gx = 0.0f, gy = 0.0f, gz = 0.0f;
        if (imu_ok) {
            imu_valid = readImu(ax, ay, az, gx, gy, gz);
        }

        // ── 4. Read magnetometer (20 Hz) ───────────────────────
        readCompass(now);
        updateYawEstimate(now);

        // ── 5. Update shared state under mutex ───────────────────
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        if (imu_valid) {
            g_imu_data.accel_x = ax;
            g_imu_data.accel_y = ay;
            g_imu_data.accel_z = az;
            g_imu_data.gyro_x  = gx;
            g_imu_data.gyro_y  = gy;
            g_imu_data.gyro_z  = gz;
        }
        g_imu_data.valid = imu_valid;

        if (mag_ok) {
            g_mag_data.mag_x   = getMagX();
            g_mag_data.mag_y   = getMagY();
            g_mag_data.mag_z   = getMagZ();
        }
        g_mag_data.valid = mag_ok;
        xSemaphoreGive(g_state_mutex);

        // ── 6. LED status ─────────────────────────────────────────
        if (g_agent_connected) {
            setStatusLed(LedCode::AGENT_CONNECTED);
        } else {
            setStatusLed(LedCode::WAITING_AGENT);
        }

        // ── 7. Pet watchdog ──────────────────────────────────────
        esp_task_wdt_reset();
    }
}
