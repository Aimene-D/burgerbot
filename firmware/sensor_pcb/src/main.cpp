#include <Arduino.h>
#include "config.h"
#include "tasks.h"
#include "microros_task.h"
#include "led.h"
#include "status_codes.h"
#include <esp_task_wdt.h>

// ── FreeRTOS mutex for shared state ──────────────────────────────
SemaphoreHandle_t g_state_mutex = NULL;

// ═════════════════════════════════════════════════════════════════
// setup()
// ═════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);

    // ── Status LED ──────────────────────────────────────────────
    initStatusLed();
    setStatusLedImmediate(LedCode::BOOTING);

    // ── Mutex ───────────────────────────────────────────────────
    g_state_mutex = xSemaphoreCreateMutex();

    // ── Watchdog (only imuTask subscribes) ──────────────────────
    esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);  // panic on expiry

    // ── FreeRTOS tasks ──────────────────────────────────────────
    xTaskCreatePinnedToCore(
        imuTask, "imu", 4096, NULL, 5, NULL, 0            // core 0, pri 5
    );
    xTaskCreatePinnedToCore(
        ledTask, "led", 2048, NULL, 1, NULL, 1             // core 1, pri 1
    );
    xTaskCreatePinnedToCore(
        microrosTask, "uros", 8192, NULL, 3, NULL, 1       // core 1, pri 3
    );
}

// ═════════════════════════════════════════════════════════════════
// loop() — idle, all work in FreeRTOS tasks
// ═════════════════════════════════════════════════════════════════
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
