#pragma once
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

void initOdometry();
// Runs one iteration: read PCNT → RPM → odometry → PID → PWM
// Targets passed in (pre-read under mutex by controlTask).
// Results stored in g_m1_measured_rpm, g_m2_measured_rpm, g_odom.
void runControlStep(uint32_t dt_ms, float m1_target_rpm, float m2_target_rpm);
