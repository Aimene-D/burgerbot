#pragma once
#include "config.h"
#include "odometry.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ── Shared state (all access guarded by g_state_mutex) ──────────
extern SemaphoreHandle_t g_state_mutex;

// Written by micro-ROS callbacks, read by controlTask
extern float     g_m1_target_rpm;
extern float     g_m2_target_rpm;
extern uint32_t  g_last_cmd_ms;
extern bool      g_agent_connected;

// Written by controlTask after runControlStep, read by microrosTask
extern OdomState s_telemetry_odom;
extern float     s_telemetry_m1_rpm;
extern float     s_telemetry_m2_rpm;
extern float     s_telemetry_m1_tgt_rpm;
extern float     s_telemetry_m2_tgt_rpm;

// Written by microrosTask /pid_config callback, read by controlTask
extern float     g_pid_config_kp;
extern float     g_pid_config_ki;
extern float     g_pid_config_kd;
extern bool      g_pid_config_pending;

// ── FreeRTOS task functions ─────────────────────────────────────
void controlTask(void* pvParams);
