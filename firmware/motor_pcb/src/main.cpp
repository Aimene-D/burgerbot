#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "odometry.h"
#include "pid.h"
#include "tasks.h"
#include "microros_task.h"
#include "led.h"
#include "status_codes.h"
#include <esp_task_wdt.h>

// ── PID instances ────────────────────────────────────────────────
PidController g_m1_pid(M1_KP, M1_KI, M1_KD);
PidController g_m2_pid(M2_KP, M2_KI, M2_KD);

// ── FreeRTOS mutex for shared state ──────────────────────────────
SemaphoreHandle_t g_state_mutex = NULL;

// ═════════════════════════════════════════════════════════════════
// setup()
// ═════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);

    // ── Hardware init ───────────────────────────────────────────
    initMotorPins();
    initEncoders();
    initOdometry();
    loadPidGainsFromNvsAndApply();
    loadStictionFromNvs();

    // ── Status LED ──────────────────────────────────────────────
    initStatusLed();
    setStatusLedImmediate(LedCode::BOOTING);

    // ── Stiction calibration (first boot or forced) ─────────────
    // If any threshold is zero, run calibration
    bool needs_calibration = false;
    for (int m = 1; m <= 2 && !needs_calibration; m++) {
        for (int d = 0; d <= 1 && !needs_calibration; d++) {
            if (getStictionThresholdPwm(m, d == 1) < 0.01f) {
                needs_calibration = true;
            }
        }
    }
    if (needs_calibration) {
        setStatusLedImmediate(LedCode::MAG_CALIB);  // purple during calibration
        calibrateStiction();
    }
    setStatusLedImmediate(LedCode::WAITING_AGENT);

    // ── Mutex ───────────────────────────────────────────────────
    g_state_mutex = xSemaphoreCreateMutex();

    // ── Watchdog (only controlTask subscribes) ───────────────────
    esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);  // panic on expiry

    // ── FreeRTOS tasks ──────────────────────────────────────────
    xTaskCreatePinnedToCore(
        controlTask, "control", 3072, NULL, 5, NULL, 0   // core 0, pri 5
    );
    xTaskCreatePinnedToCore(
        ledTask, "led", 2048, NULL, 1, NULL, 1            // core 1, pri 1
    );
    xTaskCreatePinnedToCore(
        microrosTask, "uros", 8192, NULL, 3, NULL, 1      // core 1, pri 3
    );
}

// ═════════════════════════════════════════════════════════════════
// loop() — idle, all work in FreeRTOS tasks
// ═════════════════════════════════════════════════════════════════
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
