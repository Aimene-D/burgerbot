#pragma once
#include <Arduino.h>
#include "config.h"

extern volatile int32_t g_m1_encoder_count;
extern volatile int32_t g_m2_encoder_count;

void initEncoders();
