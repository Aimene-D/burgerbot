#include "motors.h"

float g_m1_target_rpm = 0.0f;
float g_m2_target_rpm = 0.0f;
uint32_t g_last_cmd_ms = 0;

void initMotorPinsAndPwm() {
  pinMode(M1_REN_PIN, OUTPUT);
  pinMode(M1_LEN_PIN, OUTPUT);
  pinMode(M2_REN_PIN, OUTPUT);
  pinMode(M2_LEN_PIN, OUTPUT);

  digitalWrite(M1_REN_PIN, HIGH);
  digitalWrite(M1_LEN_PIN, HIGH);
  digitalWrite(M2_REN_PIN, HIGH);
  digitalWrite(M2_LEN_PIN, HIGH);

  ledcSetup(PWM_M1_R_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(PWM_M1_L_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(PWM_M2_R_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(PWM_M2_L_CH, PWM_FREQ_HZ, PWM_RES_BITS);

  ledcAttachPin(M1_RPWM_PIN, PWM_M1_R_CH);
  ledcAttachPin(M1_LPWM_PIN, PWM_M1_L_CH);
  ledcAttachPin(M2_RPWM_PIN, PWM_M2_R_CH);
  ledcAttachPin(M2_LPWM_PIN, PWM_M2_L_CH);

  stopMotors();
}

void stopMotors() {
  ledcWrite(PWM_M1_R_CH, 0);
  ledcWrite(PWM_M1_L_CH, 0);
  ledcWrite(PWM_M2_R_CH, 0);
  ledcWrite(PWM_M2_L_CH, 0);
}

void applyMotorCommand(int motor_index, int pwm_signed) {
  pwm_signed = constrain(pwm_signed, -PWM_MAX, PWM_MAX);
  if (abs(pwm_signed) < PWM_DEADBAND) pwm_signed = 0;

  if (motor_index == 1) {
    const int cmd = M1_MOTOR_DIR_INVERTED ? -pwm_signed : pwm_signed;
    const bool forward = cmd >= 0;
    const int duty = abs(cmd);
    ledcWrite(PWM_M1_R_CH, forward ? duty : 0);
    ledcWrite(PWM_M1_L_CH, forward ? 0 : duty);
  } else {
    const int cmd = M2_MOTOR_DIR_INVERTED ? -pwm_signed : pwm_signed;
    const bool forward = cmd >= 0;
    const int duty = abs(cmd);
    ledcWrite(PWM_M2_R_CH, forward ? duty : 0);
    ledcWrite(PWM_M2_L_CH, forward ? 0 : duty);
  }
}

void setTargetsFromCmdVel(float linear_x_mps, float angular_z_radps) {
  if (fabsf(linear_x_mps) < ZERO_CMD_MPS_EPS)   linear_x_mps   = 0.0f;
  if (fabsf(angular_z_radps) < ZERO_CMD_RADPS_EPS) angular_z_radps = 0.0f;

  const float v_left  = linear_x_mps - (angular_z_radps * WHEEL_BASE_M * 0.5f);
  const float v_right = linear_x_mps + (angular_z_radps * WHEEL_BASE_M * 0.5f);

  g_m1_target_rpm = (v_left  / (2.0f * static_cast<float>(M_PI) * WHEEL_RADIUS_M)) * 60.0f;
  g_m2_target_rpm = (v_right / (2.0f * static_cast<float>(M_PI) * WHEEL_RADIUS_M)) * 60.0f;
}
