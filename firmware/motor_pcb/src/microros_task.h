#pragma once
#include "config.h"
#include "odometry.h"
#include "tasks.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ── ROS 2 topic names ───────────────────────────────────────────
constexpr const char* TOPIC_CMD_VEL           = "/cmd_vel";
constexpr const char* TOPIC_WHEEL_ODOM        = "/wheel_odom";
constexpr const char* TOPIC_WHEEL_SPEEDS      = "/wheel_speeds";
constexpr const char* TOPIC_PID_CONFIG        = "/pid_config";
constexpr const char* TOPIC_CALIB_STICTION    = "/calibrate_stiction";

// ── micro-ROS task ──────────────────────────────────────────────
void microrosTask(void* pvParams);
