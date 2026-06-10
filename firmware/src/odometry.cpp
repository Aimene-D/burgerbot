#include "odometry.h"
#include "motors.h"
#include "encoders.h"
#include "pid.h"
#include "imu.h"

OdomState g_odom;
float g_m1_measured_rpm = 0.0f;
float g_m2_measured_rpm = 0.0f;

extern PidController g_m1_pid;
extern PidController g_m2_pid;

void runControlStep(uint32_t dt_ms) {
  static int32_t prev_m1_count  = 0;
  static int32_t prev_m2_count  = 0;
  static int32_t accum_m1_delta = 0;
  static int32_t accum_m2_delta = 0;
  static uint32_t accum_dt_ms   = 0;

  noInterrupts();
  const int32_t m1_count = g_m1_encoder_count;
  const int32_t m2_count = g_m2_encoder_count;
  interrupts();

  int32_t m1_delta = m1_count - prev_m1_count;
  int32_t m2_delta = m2_count - prev_m2_count;
  const float dt_s = static_cast<float>(dt_ms) / 1000.0f;

  const float max_counts_f =
      ((MAX_PLAUSIBLE_RPM / 60.0f) * COUNTS_PER_OUTPUT_REV * dt_s * 1.5f) + 1.0f;
  const int32_t max_delta_counts = static_cast<int32_t>(max_counts_f);

  if (abs(m1_delta) > max_delta_counts) m1_delta = 0;
  if (abs(m2_delta) > max_delta_counts) m2_delta = 0;

  prev_m1_count = m1_count;
  prev_m2_count = m2_count;

  accum_m1_delta += m1_delta;
  accum_m2_delta += m2_delta;
  accum_dt_ms    += dt_ms;

  if (accum_dt_ms >= RPM_ESTIMATION_WINDOW_MS) {
    const float window_dt_s = static_cast<float>(accum_dt_ms) / 1000.0f;
    const float rpm_factor  = (60.0f / COUNTS_PER_OUTPUT_REV) / window_dt_s;

    float m1_rpm_raw = static_cast<float>(accum_m1_delta) * rpm_factor;
    float m2_rpm_raw = static_cast<float>(accum_m2_delta) * rpm_factor;

    if (fabsf(m1_rpm_raw) > MAX_PLAUSIBLE_RPM) m1_rpm_raw = 0.0f;
    if (fabsf(m2_rpm_raw) > MAX_PLAUSIBLE_RPM) m2_rpm_raw = 0.0f;

    g_m1_measured_rpm = (SPEED_LPF_ALPHA * m1_rpm_raw) + ((1.0f - SPEED_LPF_ALPHA) * g_m1_measured_rpm);
    g_m2_measured_rpm = (SPEED_LPF_ALPHA * m2_rpm_raw) + ((1.0f - SPEED_LPF_ALPHA) * g_m2_measured_rpm);

    accum_m1_delta = 0;
    accum_m2_delta = 0;
    accum_dt_ms    = 0;
  }

  if (fabsf(g_m1_measured_rpm) < RPM_NOISE_EPS) g_m1_measured_rpm = 0.0f;
  if (fabsf(g_m2_measured_rpm) < RPM_NOISE_EPS) g_m2_measured_rpm = 0.0f;

  const float v_l_mps = (g_m1_measured_rpm * 2.0f * static_cast<float>(M_PI) * WHEEL_RADIUS_M) / 60.0f;
  const float v_r_mps = (g_m2_measured_rpm * 2.0f * static_cast<float>(M_PI) * WHEEL_RADIUS_M) / 60.0f;
  g_odom.linear_mps   = 0.5f * (v_r_mps + v_l_mps);
  g_odom.angular_radps = (v_l_mps - v_r_mps) / WHEEL_BASE_M;

  const float theta_mid = g_odom.theta_rad + (0.5f * g_odom.angular_radps * dt_s);
  g_odom.x_m       += g_odom.linear_mps * cosf(theta_mid) * dt_s;
  g_odom.y_m       += g_odom.linear_mps * sinf(theta_mid) * dt_s;
  g_odom.theta_rad  = normalizeAngleRad(g_odom.theta_rad + (g_odom.angular_radps * dt_s));

  if ((millis() - g_last_cmd_ms) > CMD_TIMEOUT_MS) {
    g_m1_target_rpm = 0.0f;
    g_m2_target_rpm = 0.0f;
  }

  // Command-active tracking: must run BEFORE stop_requested return so that
  // prev_cmd_active is kept accurate even between commands.
  static bool     prev_cmd_active = false;
  const bool      cmd_active_now  =
      fabsf(g_m1_target_rpm) > ZERO_CMD_RPM_EPS ||
      fabsf(g_m2_target_rpm) > ZERO_CMD_RPM_EPS;

  const bool stop_requested =
      (fabsf(g_m1_target_rpm) <= ZERO_CMD_RPM_EPS) &&
      (fabsf(g_m2_target_rpm) <= ZERO_CMD_RPM_EPS);

  if (stop_requested) {
    g_m1_target_rpm = 0.0f;
    g_m2_target_rpm = 0.0f;
    // Keep PID state — integral persists across timeouts for immediate restart
    stopMotors();
    prev_cmd_active = false; // ← ensure next command triggers the timed clamp
    return;
  }

  int m1_pwm = static_cast<int>(g_m1_pid.Update(g_m1_target_rpm, g_m1_measured_rpm, dt_s));
  int m2_pwm = static_cast<int>(g_m2_pid.Update(g_m2_target_rpm, g_m2_measured_rpm, dt_s));

  // One-shot kickstart: boost both motors to MIN_PWM_START during the first
  // 200 ms of a new command, just enough to break static friction. After that
  // the PID runs freely — no persistent clamp or conditional re-engagement.
  static uint32_t kickstart_ms = 0;
  if (cmd_active_now && !prev_cmd_active) kickstart_ms = millis();
  prev_cmd_active = cmd_active_now;

  constexpr uint32_t KICKSTART_DURATION_MS = 200;
  if (cmd_active_now && (millis() - kickstart_ms) < KICKSTART_DURATION_MS) {
    if (abs(m1_pwm) < MIN_PWM_START) m1_pwm = (m1_pwm >= 0) ? MIN_PWM_START : -MIN_PWM_START;
    if (abs(m2_pwm) < MIN_PWM_START) m2_pwm = (m2_pwm >= 0) ? MIN_PWM_START : -MIN_PWM_START;
  }

  applyMotorCommand(1, m1_pwm);
  applyMotorCommand(2, m2_pwm);
}
