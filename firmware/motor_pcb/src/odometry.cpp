#include "odometry.h"
#include "encoders.h"
#include "motors.h"
#include "pid.h"
#include <Arduino.h>
#include <driver/pcnt.h>
#include <math.h>

OdomState g_odom;
float g_m1_measured_rpm = 0.0f;
float g_m2_measured_rpm = 0.0f;

extern PidController g_m1_pid;
extern PidController g_m2_pid;

// Normalize angle without depending on imu.h
static float normalizeAngleRad(float a) {
    while (a > static_cast<float>(M_PI))  a -= 2.0f * static_cast<float>(M_PI);
    while (a < -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
    return a;
}

void initOdometry() {
    g_odom = {};
    g_m1_measured_rpm = 0.0f;
    g_m2_measured_rpm = 0.0f;
}

void runControlStep(uint32_t dt_ms, float m1_target_rpm, float m2_target_rpm) {
    static int32_t  prev_m1_count   = 0;
    static int32_t  prev_m2_count   = 0;
    static int32_t  accum_m1_delta  = 0;
    static int32_t  accum_m2_delta  = 0;
    static uint32_t accum_dt_ms     = 0;
    static float    prev_m1_target  = 0.0f;
    static float    prev_m2_target  = 0.0f;

    const float dt_s = static_cast<float>(dt_ms) / 1000.0f;

    // ── 1. Read PCNT (atomic, no ISR guard needed) ──────────────
    int16_t raw1 = 0, raw2 = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &raw1);
    pcnt_get_counter_value(PCNT_UNIT_1, &raw2);
    if (M1_ENCODER_INVERT) raw1 = -raw1;
    if (M2_ENCODER_INVERT) raw2 = -raw2;
    g_m1_encoder_count = raw1;
    g_m2_encoder_count = raw2;

    // ── 2. Compute deltas with plausibility check ────────────────
    int32_t m1_delta = raw1 - prev_m1_count;
    int32_t m2_delta = raw2 - prev_m2_count;
    prev_m1_count = raw1;
    prev_m2_count = raw2;

    // Reject implausible deltas (e.g. overflow or missed ticks)
    const float max_counts_f = ((MAX_PLAUSIBLE_RPM / 60.0f) * COUNTS_PER_OUTPUT_REV * dt_s * 1.5f) + 1.0f;
    const int32_t max_delta = static_cast<int32_t>(max_counts_f);
    if (abs(m1_delta) > max_delta) m1_delta = 0;
    if (abs(m2_delta) > max_delta) m2_delta = 0;

    accum_m1_delta += m1_delta;
    accum_m2_delta += m2_delta;
    accum_dt_ms    += dt_ms;

    // ── 3. RPM estimation (windowed) ────────────────────────────
    if (accum_dt_ms >= RPM_ACCUM_WINDOW_MS) {
        const float window_s = static_cast<float>(accum_dt_ms) / 1000.0f;
        const float rpm_factor = (60.0f / COUNTS_PER_OUTPUT_REV) / window_s;

        float m1_raw = static_cast<float>(accum_m1_delta) * rpm_factor;
        float m2_raw = static_cast<float>(accum_m2_delta) * rpm_factor;

        if (fabsf(m1_raw) > MAX_PLAUSIBLE_RPM) m1_raw = 0.0f;
        if (fabsf(m2_raw) > MAX_PLAUSIBLE_RPM) m2_raw = 0.0f;

        // LPF
        g_m1_measured_rpm = (SPEED_LPF_ALPHA * m1_raw) + ((1.0f - SPEED_LPF_ALPHA) * g_m1_measured_rpm);
        g_m2_measured_rpm = (SPEED_LPF_ALPHA * m2_raw) + ((1.0f - SPEED_LPF_ALPHA) * g_m2_measured_rpm);

        accum_m1_delta = 0;
        accum_m2_delta = 0;
        accum_dt_ms    = 0;
    }

    // Floor noise
    if (fabsf(g_m1_measured_rpm) < RPM_NOISE_EPS) g_m1_measured_rpm = 0.0f;
    if (fabsf(g_m2_measured_rpm) < RPM_NOISE_EPS) g_m2_measured_rpm = 0.0f;

    // ── 4. Odometry integration (mid-point method) ──────────────
    // m1 = right wheel, m2 = left wheel (see setTargetsFromCmdVel)
    const float v_l_mps = (g_m2_measured_rpm * 2.0f * static_cast<float>(M_PI) * WHEEL_RADIUS_M) / 60.0f;
    const float v_r_mps = (g_m1_measured_rpm * 2.0f * static_cast<float>(M_PI) * WHEEL_RADIUS_M) / 60.0f;
    g_odom.linear_mps    = 0.5f * (v_r_mps + v_l_mps);
    g_odom.angular_radps = (v_r_mps - v_l_mps) / WHEEL_BASE_M;

    const float theta_mid = g_odom.theta_rad + (0.5f * g_odom.angular_radps * dt_s);
    g_odom.x_m       += g_odom.linear_mps * cosf(theta_mid) * dt_s;
    g_odom.y_m       += g_odom.linear_mps * sinf(theta_mid) * dt_s;
    g_odom.theta_rad  = normalizeAngleRad(g_odom.theta_rad + (g_odom.angular_radps * dt_s));
    g_odom.timestamp_ms = millis();  // capture sample time for timestamp backdating

    // ── 5. Stop check (zero targets → brake) ────────────────────
    const bool stop_requested =
        (fabsf(m1_target_rpm) <= ZERO_CMD_RPM_EPS) &&
        (fabsf(m2_target_rpm) <= ZERO_CMD_RPM_EPS);

    if (stop_requested) {
        g_m1_pid.Reset();
        g_m2_pid.Reset();
        stopMotors();
        prev_m1_target = 0.0f;
        prev_m2_target = 0.0f;
        return;
    }

    // ── 6. Slew limiting ─────────────────────────────────────────
    const float max_step = MAX_RPM_SLEW_RATE * dt_s;
    float slew_m1 = constrain(m1_target_rpm, prev_m1_target - max_step, prev_m1_target + max_step);
    float slew_m2 = constrain(m2_target_rpm, prev_m2_target - max_step, prev_m2_target + max_step);
    prev_m1_target = slew_m1;
    prev_m2_target = slew_m2;

    // ── 7. Feedforward + PID ────────────────────────────────────
    const float pwm_max_f = static_cast<float>(PWM_MAX);

    // Feedforward term: maps target RPM to open-loop PWM
    float ff_m1 = (slew_m1 / MAX_RPM_AT_FULL_DUTY) * K_FF * pwm_max_f;
    float ff_m2 = (slew_m2 / MAX_RPM_AT_FULL_DUTY) * K_FF * pwm_max_f;

    // PID output is in PWM units (-PWM_MAX to +PWM_MAX)
    float pid_m1 = g_m1_pid.Update(slew_m1, g_m1_measured_rpm, dt_s);
    float pid_m2 = g_m2_pid.Update(slew_m2, g_m2_measured_rpm, dt_s);

    // Combine: FF carries steady state, PID corrects error
    // Anti-windup: if combined output saturates, PID integral was already clamped
    int m1_pwm = constrain(static_cast<int>(ff_m1 + pid_m1), -PWM_MAX, PWM_MAX);
    int m2_pwm = constrain(static_cast<int>(ff_m2 + pid_m2), -PWM_MAX, PWM_MAX);

    // ── 8. Apply ─────────────────────────────────────────────────
    applyMotorCommand(1, m1_pwm);
    applyMotorCommand(2, m2_pwm);
}
