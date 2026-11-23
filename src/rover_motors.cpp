#include "rover_motors.h"
#include "config_pins.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <cstdio>

// 150 MHz / clkdiv 15 / (PWM_MAX + 1) ≈ 10 kHz
#define PWM_MAX    1000

static uint slice_l_in3, slice_l_in4;
static uint slice_r_in1, slice_r_in2;
static uint chan_l_in3,  chan_l_in4;
static uint chan_r_in1,  chan_r_in2;

static void setup_pwm_pin(uint pin, uint* slice, uint* chan) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    *slice = pwm_gpio_to_slice_num(pin);
    *chan  = pwm_gpio_to_channel(pin);
    pwm_set_clkdiv(*slice, 15.0f);
    pwm_set_wrap(*slice, PWM_MAX);
    pwm_set_chan_level(*slice, *chan, 0);
    pwm_set_enabled(*slice, true);
}

// ── Debug helper — print full slice state ────────────────────────────────────
static void dump_slice(const char* label, uint pin, uint slice, uint chan) {
    // Read the current duty level for this channel
    // pwm_hw->slice[s].cc holds both channels: bits 0-15 = chan A, bits 16-31 = chan B
    uint32_t cc = pwm_hw->slice[slice].cc;
    uint16_t level_a = (uint16_t)(cc & 0xFFFF);
    uint16_t level_b = (uint16_t)(cc >> 16);
    uint16_t my_level = (chan == 0) ? level_a : level_b;

    bool enabled = (pwm_hw->slice[slice].csr & PWM_CH0_CSR_EN_BITS) != 0;
    uint32_t wrap = pwm_hw->slice[slice].top;
    uint32_t div  = pwm_hw->slice[slice].div;

    printf("[MOTOR_DBG] %s GP%u slice%u chan%u | "
           "enabled=%d wrap=%lu div=0x%08lX level=%u (A=%u B=%u)\n",
           label, pin, slice, chan,
           (int)enabled, (unsigned long)wrap,
           (unsigned long)div, my_level,
           level_a, level_b);
    stdio_flush();
}

void motors_dump_state(const char* label) {
    printf("[MOTOR_DBG] -- %s --\n", label);
    dump_slice("L_IN3", MOTOR_L_PIN_IN3, slice_l_in3, chan_l_in3);
    dump_slice("L_IN4", MOTOR_L_PIN_IN4, slice_l_in4, chan_l_in4);
    dump_slice("R_IN1", MOTOR_R_PIN_IN1, slice_r_in1, chan_r_in1);
    dump_slice("R_IN2", MOTOR_R_PIN_IN2, slice_r_in2, chan_r_in2);
    printf("[MOTOR_DBG] GPIO funcs: L_IN3(GP%d)=%d L_IN4(GP%d)=%d R_IN1(GP%d)=%d R_IN2(GP%d)=%d\n",
           MOTOR_L_PIN_IN3, gpio_get_function(MOTOR_L_PIN_IN3),
           MOTOR_L_PIN_IN4, gpio_get_function(MOTOR_L_PIN_IN4),
           MOTOR_R_PIN_IN1, gpio_get_function(MOTOR_R_PIN_IN1),
           MOTOR_R_PIN_IN2, gpio_get_function(MOTOR_R_PIN_IN2));
    stdio_flush();
}

void motors_init() {
    setup_pwm_pin(MOTOR_L_PIN_IN3, &slice_l_in3, &chan_l_in3);
    setup_pwm_pin(MOTOR_L_PIN_IN4, &slice_l_in4, &chan_l_in4);
    setup_pwm_pin(MOTOR_R_PIN_IN1, &slice_r_in1, &chan_r_in1);
    setup_pwm_pin(MOTOR_R_PIN_IN2, &slice_r_in2, &chan_r_in2);

    printf("[MOTOR] L_IN3: GP%d slice%d chan%d\n",
           MOTOR_L_PIN_IN3, slice_l_in3, chan_l_in3);
    printf("[MOTOR] L_IN4: GP%d slice%d chan%d\n",
           MOTOR_L_PIN_IN4, slice_l_in4, chan_l_in4);
    printf("[MOTOR] R_IN1: GP%d slice%d chan%d\n",
           MOTOR_R_PIN_IN1, slice_r_in1, chan_r_in1);
    printf("[MOTOR] R_IN2: GP%d slice%d chan%d\n",
           MOTOR_R_PIN_IN2, slice_r_in2, chan_r_in2);
    stdio_flush();

    motors_dump_state("after motors_init");
}

static void set_side(float speed,
                     uint slice_a, uint chan_a,
                     uint slice_b, uint chan_b) {
    // Inputs above 1.0 saturate at 100% duty (level > wrap holds the pin high).
    if (speed >  1.25f) speed =  1.25f;
    if (speed < -1.25f) speed = -1.25f;

    float abs_speed = (speed < 0.0f) ? -speed : speed;
    // Any nonzero command is raised to at least MIN_DUTY.
    static const float MIN_DUTY = 0.70f;
    if (abs_speed > 0.0f && abs_speed < MIN_DUTY) abs_speed = MIN_DUTY;
    uint16_t duty = (uint16_t)(abs_speed * (float)PWM_MAX);

    if (speed > 0.0f) {
        pwm_set_chan_level(slice_a, chan_a, duty);
        pwm_set_chan_level(slice_b, chan_b, 0);
    } else if (speed < 0.0f) {
        pwm_set_chan_level(slice_a, chan_a, 0);
        pwm_set_chan_level(slice_b, chan_b, duty);
    } else {
        pwm_set_chan_level(slice_a, chan_a, 0);
        pwm_set_chan_level(slice_b, chan_b, 0);
    }
}

void motors_set(float left, float right) {
    set_side(left,  slice_l_in3, chan_l_in3, slice_l_in4, chan_l_in4);
    set_side(right, slice_r_in1, chan_r_in1, slice_r_in2, chan_r_in2);
}


void motors_stop() {
    motors_set(0.0f, 0.0f);
}