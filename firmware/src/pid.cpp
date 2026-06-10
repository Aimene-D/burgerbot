#include "pid.h"
#include <Preferences.h>

static Preferences g_pid_preferences;
extern PidController g_m1_pid;
extern PidController g_m2_pid;

PidController::PidController(float kp, float ki, float kd, float ff)
    : kp_(kp), ki_(ki), kd_(kd), ff_(ff) {}

void PidController::SetGains(float kp, float ki, float kd) {
  kp_ = kp; ki_ = ki; kd_ = kd;
}

void PidController::GetGains(float& kp, float& ki, float& kd) const {
  kp = kp_; ki = ki_; kd = kd_;
}

void PidController::SetFeedforward(float ff) { ff_ = ff; }

void PidController::Reset() {
  integral_ = 0.0f;
  prev_measurement_ = 0.0f;
  first_run_ = true;
}

float PidController::Update(float setpoint, float measurement, float dt_s) {
  if (dt_s <= 0.0f) return 0.0f;
  const float error = setpoint - measurement;
  integral_ += error * dt_s;
  integral_ = constrain(integral_, -300.0f, 300.0f);
  float d_measurement = 0.0f;
  if (!first_run_) {
    d_measurement = (measurement - prev_measurement_) / dt_s;
  }
  prev_measurement_ = measurement;
  first_run_ = false;
  float output = (kp_ * error) + (ki_ * integral_) - (kd_ * d_measurement) + (ff_ * setpoint);
  output = constrain(output, -static_cast<float>(PWM_MAX), static_cast<float>(PWM_MAX));
  return output;
}

void savePidGainsToNvs(float kp, float ki, float kd) {
  if (!g_pid_preferences.begin(PID_PREFS_NAMESPACE, false)) return;
  g_pid_preferences.putFloat(PID_PREF_KEY_KP, kp);
  g_pid_preferences.putFloat(PID_PREF_KEY_KI, ki);
  g_pid_preferences.putFloat(PID_PREF_KEY_KD, kd);
  g_pid_preferences.end();
}

void loadPidGainsFromNvsAndApply() {
  if (!g_pid_preferences.begin(PID_PREFS_NAMESPACE, true)) return;
  const bool has_kp = g_pid_preferences.isKey(PID_PREF_KEY_KP);
  const bool has_ki = g_pid_preferences.isKey(PID_PREF_KEY_KI);
  const bool has_kd = g_pid_preferences.isKey(PID_PREF_KEY_KD);
  if (has_kp && has_ki && has_kd) {
    const float kp = g_pid_preferences.getFloat(PID_PREF_KEY_KP, M1_KP);
    const float ki = g_pid_preferences.getFloat(PID_PREF_KEY_KI, M1_KI);
    const float kd = g_pid_preferences.getFloat(PID_PREF_KEY_KD, M1_KD);
    g_m1_pid.SetGains(kp, ki, kd);
    g_m2_pid.SetGains(kp, ki, kd);
    g_m1_pid.Reset();
    g_m2_pid.Reset();
  }
  g_pid_preferences.end();
}

void saveMagOffsetsToNvs(float offset_x, float offset_y, float offset_z) {
  if (!g_pid_preferences.begin(MAG_PREFS_NAMESPACE, false)) return;
  g_pid_preferences.putFloat(MAG_PREF_KEY_OFFSET_X, offset_x);
  g_pid_preferences.putFloat(MAG_PREF_KEY_OFFSET_Y, offset_y);
  g_pid_preferences.putFloat(MAG_PREF_KEY_OFFSET_Z, offset_z);
  g_pid_preferences.end();
}
