#pragma once

#include <cstdint>
#include <cmath>

// ═══════════════════════════════════════════════════════════════════════
// GPIO pins — Motor PCB (CARTE MOTEUR)
// ═══════════════════════════════════════════════════════════════════════

// DRV8871 — Motor 1 (MCPWM_UNIT_0, TIMER_0)
constexpr int M1_IN1_PIN = 38;     // MCPWM_GEN_A
constexpr int M1_IN2_PIN = 39;     // MCPWM_GEN_B

// DRV8871 — Motor 2 (MCPWM_UNIT_0, TIMER_1)
constexpr int M2_IN1_PIN = 48;     // MCPWM_GEN_A
constexpr int M2_IN2_PIN = 47;     // MCPWM_GEN_B

// PCNT — Encoder 1 (PCNT_UNIT_0)
constexpr int M1_ENC_A_PIN = 37;   // pulse_gpio
constexpr int M1_ENC_B_PIN = 36;   // ctrl_gpio

// PCNT — Encoder 2 (PCNT_UNIT_1)
constexpr int M2_ENC_A_PIN = 21;   // pulse_gpio
constexpr int M2_ENC_B_PIN = 14;   // ctrl_gpio

// WS2812B RGB status LED
constexpr int STATUS_LED_PIN  = 15;
constexpr int STATUS_LED_COUNT = 1;

// ═══════════════════════════════════════════════════════════════════════
// MCPWM configuration
// ═══════════════════════════════════════════════════════════════════════

// Center-aligned (UP_DOWN counter): timer runs at 2× target PWM freq.
// Target 20 kHz → timer set to 40 kHz.
constexpr int   MCPWM_PWM_FREQ_HZ     = 20000;
constexpr int   MCPWM_TIMER_FREQ_HZ   = 40000;  // 2× for UP_DOWN mode
constexpr float MCPWM_DEADTIME_NS     = 0.0f;   // sign-magnitude: no shoot-through path

// ═══════════════════════════════════════════════════════════════════════
// Kinematics (PCNT ×4 corrected)
// ═══════════════════════════════════════════════════════════════════════

constexpr float ENCODER_PPR             = 11.0f;
constexpr float GEAR_RATIO              = 18.8f;
constexpr float ENCODER_EDGE_MULTIPLIER = 4.0f;      // PCNT quadrature ×4
constexpr float COUNTS_PER_OUTPUT_REV   = ENCODER_PPR * GEAR_RATIO * ENCODER_EDGE_MULTIPLIER;
constexpr float WHEEL_RADIUS_M          = 0.0625f;    // 62.5 mm
constexpr float WHEEL_BASE_M            = 0.282f;     // 282 mm track width

// ═══════════════════════════════════════════════════════════════════════
// Timing
// ═══════════════════════════════════════════════════════════════════════

constexpr uint32_t CONTROL_PERIOD_MS        = 1;       // 1 kHz control loop
constexpr uint32_t TELEMETRY_PERIOD_MS      = 50;      // 20 Hz telemetry
constexpr uint32_t CMD_TIMEOUT_MS           = 300;     // safety stop if no cmd
constexpr uint32_t RPM_ACCUM_WINDOW_MS      = 5;       // 5 ms RPM accumulation window
constexpr uint32_t WATCHDOG_TIMEOUT_S       = 3;       // TWDT seconds (min 1; ~3000 missed 1 kHz ticks)
constexpr bool     PUBLISH_RAW_ODOM_TF      = false;

// ═══════════════════════════════════════════════════════════════════════
// PID + Feedforward defaults
// ═══════════════════════════════════════════════════════════════════════

constexpr float M1_KP = 1.00f;
constexpr float M1_KI = 0.50f;
constexpr float M1_KD = 0.01f;
constexpr float M2_KP = 1.00f;
constexpr float M2_KI = 0.50f;
constexpr float M2_KD = 0.01f;

// Feedforward: PWM_ff = target_rpm * K_FF / MAX_RPM_AT_FULL_DUTY
// With K_FF=1.0 and MAX_RPM=700, 350 RPM target → 50% feedforward PWM
constexpr float K_FF                   = 1.0f;
constexpr float MAX_RPM_AT_FULL_DUTY   = 700.0f;

// Slew limiting
constexpr float MAX_RPM_SLEW_RATE      = 500.0f;      // RPM/s

// ═══════════════════════════════════════════════════════════════════════
// PWM / Control tuning
// ═══════════════════════════════════════════════════════════════════════

constexpr int   PWM_RES_BITS           = 10;           // MCPWM resolution
constexpr int   PWM_MAX                = (1 << PWM_RES_BITS) - 1;  // 1023
constexpr int   PWM_DEADBAND           = 8;            // minimum non-zero PWM
constexpr float SPEED_LPF_ALPHA        = 0.30f;
constexpr float ZERO_CMD_RPM_EPS       = 0.8f;
constexpr float ZERO_CMD_MPS_EPS       = 0.01f;
constexpr float ZERO_CMD_RADPS_EPS     = 0.05f;
constexpr float RPM_NOISE_EPS          = 0.05f;
constexpr float MAX_PLAUSIBLE_RPM      = 700.0f;

// ═══════════════════════════════════════════════════════════════════════
// Stiction calibration
// ═══════════════════════════════════════════════════════════════════════

constexpr float    STICTION_PWM_STEP         = 0.02f;
constexpr uint32_t STICTION_STEP_DELAY_MS    = 80;
constexpr int      STICTION_COUNT_THRESHOLD  = 3;
constexpr int      STICTION_SWEEPS_PER_DIR   = 4;
constexpr float    STICTION_SAFETY_MARGIN    = 1.04f;

// ═══════════════════════════════════════════════════════════════════════
// NVS keys
// ═══════════════════════════════════════════════════════════════════════

constexpr const char* PID_PREFS_NAMESPACE       = "pid";
constexpr const char* PID_PREF_KEY_KP           = "kp";
constexpr const char* PID_PREF_KEY_KI           = "ki";
constexpr const char* PID_PREF_KEY_KD           = "kd";

constexpr const char* STICTION_PREFS_NAMESPACE    = "stick";
constexpr const char* STICTION_PREF_KEY_M1_FWD   = "m1f";
constexpr const char* STICTION_PREF_KEY_M1_REV   = "m1r";
constexpr const char* STICTION_PREF_KEY_M2_FWD   = "m2f";
constexpr const char* STICTION_PREF_KEY_M2_REV   = "m2r";
constexpr const char* STICTION_PREF_KEY_TRIGGER  = "trig";

// ═══════════════════════════════════════════════════════════════════════
// ═══════════════════════════════════════════════════════════════════════
// micro-ROS
// ═══════════════════════════════════════════════════════════════════════

constexpr int  MICROROS_RETRY_ATTEMPTS       = 30;    // 30 × 200 ms = 6 s
constexpr int  MICROROS_RETRY_DELAY_MS        = 200;
constexpr int  MICROROS_RECONNECT_BACKOFF_MS  = 2000;  // 2 s between full retry cycles

// Motor direction inversion (set via build flags or defaults)
// ═══════════════════════════════════════════════════════════════════════

#ifndef M1_MOTOR_DIR_INVERT
#define M1_MOTOR_DIR_INVERT 1
#endif
#ifndef M2_MOTOR_DIR_INVERT
#define M2_MOTOR_DIR_INVERT 0
#endif

constexpr bool M1_MOTOR_DIR_INVERTED = (M1_MOTOR_DIR_INVERT != 0);
constexpr bool M2_MOTOR_DIR_INVERTED = (M2_MOTOR_DIR_INVERT != 0);
constexpr bool M1_ENCODER_INVERT = true;
constexpr bool M2_ENCODER_INVERT = true;
