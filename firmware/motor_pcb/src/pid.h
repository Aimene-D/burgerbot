#pragma once
#include "config.h"

class PidController {
public:
    PidController(float kp, float ki, float kd);
    void SetGains(float kp, float ki, float kd);
    void GetGains(float& kp, float& ki, float& kd) const;
    void Reset();
    float Update(float setpoint, float measurement, float dt_s);

private:
    float kp_, ki_, kd_;
    float integral_ = 0.0f;
    float prev_measurement_ = 0.0f;
    bool  first_run_ = true;
};

// NVS persistence for PID gains
void savePidGainsToNvs(float kp, float ki, float kd);
void loadPidGainsFromNvsAndApply();
