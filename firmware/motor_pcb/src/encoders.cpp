#include "encoders.h"
#include <driver/pcnt.h>

// Shared with odometry (read at 1 kHz by controlTask)
volatile int32_t g_m1_encoder_count = 0;
volatile int32_t g_m2_encoder_count = 0;

void initEncoders() {
    // ── Encoder 1: PCNT_UNIT_0 ──────────────────────────────────
    // ×4 quadrature: count both edges on pulse pin, control level reverses direction
    pcnt_config_t pcnt_1 = {};
    pcnt_1.pulse_gpio_num = M1_ENC_A_PIN;
    pcnt_1.ctrl_gpio_num  = M1_ENC_B_PIN;
    pcnt_1.channel        = PCNT_CHANNEL_0;
    pcnt_1.unit           = PCNT_UNIT_0;
    pcnt_1.pos_mode       = PCNT_COUNT_INC;      // count up on rising edge of A
    pcnt_1.neg_mode       = PCNT_COUNT_DEC;      // count down on falling edge of A
    pcnt_1.hctrl_mode     = PCNT_MODE_REVERSE;   // reverse direction when B = HIGH
    pcnt_1.lctrl_mode     = PCNT_MODE_KEEP;      // normal direction when B = LOW
    pcnt_1.counter_h_lim  = 16000;
    pcnt_1.counter_l_lim  = -16000;
    pcnt_unit_config(&pcnt_1);

    // Glitch filter: reject pulses < ~80 ns (at 240 MHz, 100 cycles)
    pcnt_set_filter_value(PCNT_UNIT_0, 100);
    pcnt_filter_enable(PCNT_UNIT_0);

    // ── Encoder 2: PCNT_UNIT_1 ──────────────────────────────────
    pcnt_config_t pcnt_2 = {};
    pcnt_2.pulse_gpio_num = M2_ENC_A_PIN;
    pcnt_2.ctrl_gpio_num  = M2_ENC_B_PIN;
    pcnt_2.channel        = PCNT_CHANNEL_0;
    pcnt_2.unit           = PCNT_UNIT_1;
    pcnt_2.pos_mode       = PCNT_COUNT_INC;
    pcnt_2.neg_mode       = PCNT_COUNT_DEC;
    pcnt_2.hctrl_mode     = PCNT_MODE_REVERSE;
    pcnt_2.lctrl_mode     = PCNT_MODE_KEEP;
    pcnt_2.counter_h_lim  = 16000;
    pcnt_2.counter_l_lim  = -16000;
    pcnt_unit_config(&pcnt_2);

    pcnt_set_filter_value(PCNT_UNIT_1, 100);
    pcnt_filter_enable(PCNT_UNIT_1);

    // ── Start counting ───────────────────────────────────────────
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);

    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);
}

int16_t readEncoderCount(int motor) {
    int16_t count = 0;
    pcnt_unit_t unit = (motor == 1) ? PCNT_UNIT_0 : PCNT_UNIT_1;
    pcnt_get_counter_value(unit, &count);

    // Apply encoder direction inversion
    if ((motor == 1 && M1_ENCODER_INVERT) || (motor == 2 && M2_ENCODER_INVERT)) {
        count = -count;
    }

    // Update volatile globals used by odometry
    if (motor == 1) g_m1_encoder_count = count;
    else            g_m2_encoder_count = count;

    return count;
}
