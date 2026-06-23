#pragma once
#include "config.h"
#include "tasks.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ── ROS 2 topic names ───────────────────────────────────────────
constexpr const char* TOPIC_IMU_RAW         = "/imu/raw";
constexpr const char* TOPIC_IMU_MAG         = "/imu/mag";
constexpr const char* TOPIC_IMU_STATUS      = "/imu/status/valid";
constexpr const char* TOPIC_MAG_STATUS      = "/mag/status/valid";
constexpr const char* TOPIC_CALIB_MAG       = "/calibrate_mag";

// ── micro-ROS task ──────────────────────────────────────────────
void microrosTask(void* pvParams);
