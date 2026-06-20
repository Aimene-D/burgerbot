#include "encoders.h"

volatile int32_t g_m1_encoder_count = 0;
volatile int32_t g_m2_encoder_count = 0;

void IRAM_ATTR m1EncoderISR() {
  int8_t delta = (digitalRead(M1_ENC_A_PIN) == digitalRead(M1_ENC_B_PIN)) ? 1 : -1;
  if (M1_ENCODER_INVERT) delta = -delta;
  g_m1_encoder_count += delta;
}

void IRAM_ATTR m2EncoderISR() {
  int8_t delta = (digitalRead(M2_ENC_A_PIN) == digitalRead(M2_ENC_B_PIN)) ? 1 : -1;
  if (M2_ENCODER_INVERT) delta = -delta;
  g_m2_encoder_count += delta;
}

void initEncoders() {
  pinMode(M1_ENC_A_PIN, INPUT_PULLUP);
  pinMode(M1_ENC_B_PIN, INPUT_PULLUP);
  pinMode(M2_ENC_A_PIN, INPUT_PULLUP);
  pinMode(M2_ENC_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(M1_ENC_A_PIN), m1EncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A_PIN), m2EncoderISR, RISING);
}
