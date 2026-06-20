#include "tasks.h"
#include "motors.h"
#include "encoders.h"
#include "odometry.h"
#include "pid.h"
#include "led.h"
#include "status_codes.h"
#include <Arduino.h>
#include <esp_task_wdt.h>

// PID instances (defined in main.cpp)
extern PidController g_m1_pid;
extern PidController g_m2_pid;

// ── Shared state definitions ────────────────────────────────────
float     g_m1_target_rpm = 0.0f;
float     g_m2_target_rpm = 0.0f;
uint32_t  g_last_cmd_ms   = 0;
bool      g_agent_connected = false;

// ── Telemetry copies (filled under mutex by controlTask, read by microrosTask)
OdomState s_telemetry_odom;
float s_telemetry_m1_rpm = 0.0f;
float s_telemetry_m2_rpm = 0.0f;

// ═══════════════════════════════════════════════════════════════
// controlTask — Core 0, priority 5, 1 kHz hard real-time
// ═══════════════════════════════════════════════════════════════
void controlTask(void* pvParams) {
    (void)pvParams;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

    // Subscribe this task to watchdog
    esp_task_wdt_add(NULL);

    while (true) {
        vTaskDelayUntil(&last_wake, period);  // 1 ms tick

        // ── 1. Read targets under mutex ─────────────────────────
        float tgt1, tgt2;
        uint32_t last_cmd;
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        tgt1    = g_m1_target_rpm;
        tgt2    = g_m2_target_rpm;
        last_cmd = g_last_cmd_ms;
        xSemaphoreGive(g_state_mutex);

        // ── 2. Command timeout check ────────────────────────────
        if ((millis() - last_cmd) > CMD_TIMEOUT_MS) {
            stopMotors();
            g_m1_pid.Reset();
            g_m2_pid.Reset();
            setStatusLed(LedCode::CMD_TIMEOUT);
            esp_task_wdt_reset();
            continue;
        }

        // ── 3. Run control step (PCNT → RPM → odom → PID → PWM) ─
        runControlStep(CONTROL_PERIOD_MS, tgt1, tgt2);

        // ── 4. Update shared measured state for telemetry ───────
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        s_telemetry_m1_rpm = g_m1_measured_rpm;
        s_telemetry_m2_rpm = g_m2_measured_rpm;
        s_telemetry_odom   = g_odom;
        xSemaphoreGive(g_state_mutex);

        // ── 5. LED status (don't drown out microrosTask when disconnected) ──
        if (!g_agent_connected) {
            // microrosTask handles WAITING_AGENT / ERROR
            // Only override on CMD_TIMEOUT so red blink is visible
            if ((millis() - last_cmd) > CMD_TIMEOUT_MS) {
                setStatusLed(LedCode::CMD_TIMEOUT);
            }
        } else {
            bool spinning = (fabsf(g_m1_measured_rpm) > RPM_NOISE_EPS ||
                             fabsf(g_m2_measured_rpm) > RPM_NOISE_EPS);
            setStatusLed(spinning ? LedCode::CONTROL_ACTIVE : LedCode::AGENT_CONNECTED);
        }

        // ── 6. Pet watchdog ─────────────────────────────────────
        esp_task_wdt_reset();
    }
}

