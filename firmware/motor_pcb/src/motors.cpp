#include "motors.h"
#include "encoders.h"
#include <Arduino.h>
#include <Preferences.h>
#include <driver/mcpwm.h>
#include <driver/pcnt.h>

static Preferences g_stiction_prefs;
static float g_stiction_threshold[2][2] = {{0,0},{0,0}};  // [motor][rev] in PWM counts

// ── Init: MCPWM timers, generators, GPIO ──────────────────────────
void initMotorPins() {
    // Both motors on MCPWM_UNIT_0 (3 timers, same clock domain — inherently synced)
    // Motor 1: TIMER_0, GEN_A = IN1, GEN_B = IN2
    // Motor 2: TIMER_1, GEN_A = IN1, GEN_B = IN2

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, M1_IN1_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, M1_IN2_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, M2_IN1_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, M2_IN2_PIN);

    // Center-aligned mode: UP_DOWN_COUNTER halves effective freq, so set timer to 2×
    mcpwm_config_t cfg = {};
    cfg.frequency    = static_cast<float>(MCPWM_TIMER_FREQ_HZ);
    cfg.cmpr_a       = 0.0f;
    cfg.cmpr_b       = 0.0f;
    cfg.counter_mode = MCPWM_UP_DOWN_COUNTER;
    cfg.duty_mode    = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &cfg);  // Motor 1
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &cfg);  // Motor 2
}

// ── Brake: both channels HIGH ─────────────────────────────────────
void stopMotors() {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 100.0f);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, 100.0f);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 100.0f);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, 100.0f);
}

// ── Apply signed PWM to one motor ─────────────────────────────────
// pwm_signed: -PWM_MAX … +PWM_MAX
// DRV8871 truth table via duty %:
//   Forward: IN1 = d%, IN2 = 0%
//   Reverse: IN1 = 0%, IN2 = d%
//   Brake:   IN1 = 100%, IN2 = 100%
//   |pwm| < deadband → brake
void applyMotorCommand(int motor, int pwm_signed) {
    if (motor == 1 && M1_MOTOR_DIR_INVERTED) pwm_signed = -pwm_signed;
    if (motor == 2 && M2_MOTOR_DIR_INVERTED) pwm_signed = -pwm_signed;

    pwm_signed = constrain(pwm_signed, -PWM_MAX, PWM_MAX);

    const bool braking = (abs(pwm_signed) < PWM_DEADBAND);
    const float duty_pct = static_cast<float>(abs(pwm_signed)) * 100.0f / static_cast<float>(PWM_MAX);
    const float stiction = getStictionThresholdPwm(motor, pwm_signed < 0);
    const float effective_duty = (duty_pct > 0.0f && duty_pct < stiction) ? stiction : duty_pct;

    mcpwm_unit_t unit = MCPWM_UNIT_0;
    mcpwm_timer_t timer = (motor == 1) ? MCPWM_TIMER_0 : MCPWM_TIMER_1;

    if (braking) {
        mcpwm_set_duty(unit, timer, MCPWM_GEN_A, 100.0f);
        mcpwm_set_duty(unit, timer, MCPWM_GEN_B, 100.0f);
    } else if (pwm_signed >= 0) {
        mcpwm_set_duty(unit, timer, MCPWM_GEN_A, effective_duty);
        mcpwm_set_duty(unit, timer, MCPWM_GEN_B, 0.0f);
    } else {
        mcpwm_set_duty(unit, timer, MCPWM_GEN_A, 0.0f);
        mcpwm_set_duty(unit, timer, MCPWM_GEN_B, effective_duty);
    }
}

// ── Differential drive kinematics ─────────────────────────────────
void setTargetsFromCmdVel(float linear_x_mps, float angular_z_radps) {
    if (fabsf(linear_x_mps)   < ZERO_CMD_MPS_EPS)   linear_x_mps   = 0.0f;
    if (fabsf(angular_z_radps) < ZERO_CMD_RADPS_EPS) angular_z_radps = 0.0f;

    const float v_left  = linear_x_mps - (angular_z_radps * WHEEL_BASE_M * 0.5f);
    const float v_right = linear_x_mps + (angular_z_radps * WHEEL_BASE_M * 0.5f);

    // m/s → RPM: RPM = (v / (2πR)) × 60
    const float rps_to_rpm = 60.0f / (2.0f * static_cast<float>(M_PI) * WHEEL_RADIUS_M);
    // These are read under mutex by controlTask
    extern float g_m1_target_rpm;
    extern float g_m2_target_rpm;
    g_m1_target_rpm = v_left  * rps_to_rpm;
    g_m2_target_rpm = v_right * rps_to_rpm;
}

// ── Stiction calibration ──────────────────────────────────────────
void calibrateStiction() {
    float results[2][2] = {{0,0},{0,0}};  // [motor][dir]

    for (int motor = 1; motor <= 2; motor++) {
        for (int dir = 0; dir <= 1; dir++) {  // 0 = fwd, 1 = rev
            float sum = 0.0f;
            for (int sweep = 0; sweep < STICTION_SWEEPS_PER_DIR; sweep++) {
                // Reset encoder
                pcnt_counter_pause((motor == 1) ? PCNT_UNIT_0 : PCNT_UNIT_1);
                pcnt_counter_clear((motor == 1) ? PCNT_UNIT_0 : PCNT_UNIT_1);
                pcnt_counter_resume((motor == 1) ? PCNT_UNIT_0 : PCNT_UNIT_1);

                int16_t baseline = 0;
                pcnt_get_counter_value((motor == 1) ? PCNT_UNIT_0 : PCNT_UNIT_1, &baseline);

                float threshold_pwm = 0.0f;
                for (int step = 1; step <= 50; step++) {
                    float duty_pct = step * STICTION_PWM_STEP * 100.0f;  // 2% per step → up to 100%
                    if (duty_pct > 100.0f) duty_pct = 100.0f;

                    mcpwm_timer_t timer = (motor == 1) ? MCPWM_TIMER_0 : MCPWM_TIMER_1;
                    if (dir == 0) {  // forward
                        mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_GEN_A, duty_pct);
                        mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_GEN_B, 0.0f);
                    } else {         // reverse
                        mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_GEN_A, 0.0f);
                        mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_GEN_B, duty_pct);
                    }

                    delay(STICTION_STEP_DELAY_MS);

                    int16_t count = 0;
                    pcnt_get_counter_value((motor == 1) ? PCNT_UNIT_0 : PCNT_UNIT_1, &count);
                    if (abs(count - baseline) >= STICTION_COUNT_THRESHOLD) {
                        threshold_pwm = duty_pct;
                        break;
                    }
                }
                sum += threshold_pwm;
                stopMotors();
                delay(200);
            }
            results[motor - 1][dir] = (sum / STICTION_SWEEPS_PER_DIR) * STICTION_SAFETY_MARGIN;
        }
    }

    // Persist to NVS
    if (g_stiction_prefs.begin(STICTION_PREFS_NAMESPACE, false)) {
        g_stiction_prefs.putFloat(STICTION_PREF_KEY_M1_FWD, results[0][0]);
        g_stiction_prefs.putFloat(STICTION_PREF_KEY_M1_REV, results[0][1]);
        g_stiction_prefs.putFloat(STICTION_PREF_KEY_M2_FWD, results[1][0]);
        g_stiction_prefs.putFloat(STICTION_PREF_KEY_M2_REV, results[1][1]);
        g_stiction_prefs.end();
    }

    g_stiction_threshold[0][0] = results[0][0];
    g_stiction_threshold[0][1] = results[0][1];
    g_stiction_threshold[1][0] = results[1][0];
    g_stiction_threshold[1][1] = results[1][1];
}

void loadStictionFromNvs() {
    if (!g_stiction_prefs.begin(STICTION_PREFS_NAMESPACE, true)) return;
    g_stiction_threshold[0][0] = g_stiction_prefs.getFloat(STICTION_PREF_KEY_M1_FWD, 0.0f);
    g_stiction_threshold[0][1] = g_stiction_prefs.getFloat(STICTION_PREF_KEY_M1_REV, 0.0f);
    g_stiction_threshold[1][0] = g_stiction_prefs.getFloat(STICTION_PREF_KEY_M2_FWD, 0.0f);
    g_stiction_threshold[1][1] = g_stiction_prefs.getFloat(STICTION_PREF_KEY_M2_REV, 0.0f);
    g_stiction_prefs.end();
}

float getStictionThresholdPwm(int motor, bool reverse) {
    int m = (motor == 1) ? 0 : 1;
    int d = reverse ? 1 : 0;
    return g_stiction_threshold[m][d];
}
