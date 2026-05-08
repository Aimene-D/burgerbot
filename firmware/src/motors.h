#pragma once
#include <Arduino.h>
#include "config.h"

void initMotorPinsAndPwm();
void stopMotors();
void applyMotorCommand(int motor_index, int pwm_signed);
void setTargetsFromCmdVel(float linear_x_mps, float angular_z_radps);

extern float g_m1_target_rpm;
extern float g_m2_target_rpm;
extern uint32_t g_last_cmd_ms;
