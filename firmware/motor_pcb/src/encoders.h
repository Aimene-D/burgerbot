#pragma once
#include "config.h"

extern volatile int32_t g_m1_encoder_count;
extern volatile int32_t g_m2_encoder_count;

void initEncoders();
int16_t readEncoderCount(int motor);    // atomic PCNT read
