#pragma once
#include "config.h"

void initMotorPins();
void stopMotors();                      // brake (IN1=IN2=100%)
void applyMotorCommand(int motor, int pwm_signed);  // signed PWM → MCPWM duty
void setTargetsFromCmdVel(float linear_x_mps, float angular_z_radps);

// Stiction calibration
void calibrateStiction();               // runs at startup, persists thresholds to NVS
void loadStictionFromNvs();             // load persisted thresholds into runtime cache
float getStictionThresholdPwm(int motor, bool reverse);  // runtime threshold lookup
void triggerStictionCalibration();      // persist boot trigger flag to NVS
bool isStictionCalibrationTriggered();  // check NVS for trigger flag
void clearStictionCalibrationTrigger(); // remove trigger flag from NVS
