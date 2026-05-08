#pragma once
#include <Arduino.h>
#include "config.h"

struct OdomState {
  float x_m         = 0.0f;
  float y_m         = 0.0f;
  float theta_rad   = 0.0f;
  float linear_mps  = 0.0f;
  float angular_radps = 0.0f;
};

extern OdomState g_odom;
extern float g_m1_measured_rpm;
extern float g_m2_measured_rpm;

void runControlStep(uint32_t dt_ms);
