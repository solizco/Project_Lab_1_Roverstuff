#include "collector_arms.h"
#include "config_pins.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "cstdio"

// MG995 pulse range
static const int PULSE_MIN_US  = 600;
static const int PULSE_MAX_US  = 2400;

static uint slice_a, slice_b;
static uint chan_a,  chan_b;
static bool initialised = false;

static uint16_t us_to_level(int us) {
    if (us < PULSE_MIN_US) us = PULSE_MIN_US;
    if (us > PULSE_MAX_US) us = PULSE_MAX_US;
    return (uint16_t)us;
}

static int angle_to_us(int angle) {
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;
    return PULSE_MIN_US + (PULSE_MAX_US - PULSE_MIN_US) * angle / 180;
}

static void set_both_angle(int angle) {
    float t = (float)angle / (float)ARMS_ROM_MAX;
    int trim = (int)((1.0f - t) * ARM_A_TRIM_BOT_DEG + t * ARM_A_TRIM_TOP_DEG);

    // Both channels are on the same PWM slice, so both new pulse widths
    // take effect on the same 50 Hz period
    pwm_set_chan_level(slice_a, chan_a, us_to_level(angle_to_us(angle + trim)));
    pwm_set_chan_level(slice_b, chan_b, us_to_level(angle_to_us(180 - angle)));
}

void arms_init() {
    // Both pins share slice 0 — configure channels before enabling
    gpio_set_function(ARM_PIN_A, GPIO_FUNC_PWM);
    slice_a = pwm_gpio_to_slice_num(ARM_PIN_A);
    chan_a  = pwm_gpio_to_channel(ARM_PIN_A);

    gpio_set_function(ARM_PIN_B, GPIO_FUNC_PWM);
    slice_b = pwm_gpio_to_slice_num(ARM_PIN_B);
    chan_b  = pwm_gpio_to_channel(ARM_PIN_B);

    // Set slice-level config once (slice_a == slice_b == slice 0)
    // clkdiv=150 → 1MHz tick, wrap=20000 → 50Hz, so 1 level = 1us
    pwm_set_clkdiv(slice_a, 150.0f);
    pwm_set_wrap(slice_a, 20000);

    
    // Set both channel levels before enabling
    pwm_set_chan_level(slice_a, chan_a, us_to_level(angle_to_us(ARMS_ROM_MAX + ARM_A_TRIM_TOP_DEG)));  // arm A includes its top-position trim
    pwm_set_chan_level(slice_b, chan_b, us_to_level(angle_to_us(180 - ARMS_ROM_MAX)));

    pwm_set_enabled(slice_a, true);

    printf("[ARMS] A: GP%d slice%d chan%d\n", ARM_PIN_A, slice_a, chan_a);
    printf("[ARMS] B: GP%d slice%d chan%d\n", ARM_PIN_B, slice_b, chan_b);
    
    stdio_flush();
    initialised = true;
}


static int current_angle = ARMS_ROM_MAX; 
int arms_get_angle() { return current_angle; }


void arms_sweep(int start_deg, int end_deg) {
    if (!initialised) return;
    int step = (end_deg >= start_deg) ? 1 : -1;
    uint32_t delay_us = (step < 0) ? ARM_DOWN_DELAY_US : ARM_UP_DELAY_US;

    for (int angle = start_deg; angle != end_deg + step; angle += step) {
        set_both_angle(angle);
        sleep_us(delay_us);
    }
    current_angle = end_deg;
}

void arms_set(int angle_deg) {
    if (!initialised) return;
    set_both_angle(angle_deg);
    current_angle = angle_deg;
}

void arms_release() {
    if (!initialised) return;
    pwm_set_chan_level(slice_a, chan_a, 0);
    pwm_set_chan_level(slice_b, chan_b, 0);
}