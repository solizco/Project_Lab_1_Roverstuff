#include "flywheel_motors.h"
#include "config_pins.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <cstdio>

// PWM wrap matches 20kHz at 150MHz sys_clk with clkdiv=1
static const uint32_t PWM_WRAP    = 7500;
// Cruise duty: any level above PWM_WRAP holds the output high (100%), which
// is what ran in competition. For a partial cruise speed use a fraction of
// PWM_WRAP instead, e.g. PWM_WRAP * 3 / 4 for 75%.
static const uint32_t DUTY_CRUISE = PWM_WRAP + 1;

void flywheel_init() {
    // Configure IN4 as PWM output, speed = 0
    gpio_set_function(FLYWHEEL_PIN_IN4, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(FLYWHEEL_PIN_IN4);
    pwm_set_clkdiv(slice, 1.0f);
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_enabled(slice, true);
    pwm_set_gpio_level(FLYWHEEL_PIN_IN4, 0);

    // IN3 = plain GPIO, held LOW (forward direction)
    gpio_init(FLYWHEEL_PIN_IN3);
    gpio_set_dir(FLYWHEEL_PIN_IN3, GPIO_OUT);
    gpio_put(FLYWHEEL_PIN_IN3, 0);

    printf("[FW] init done IN3=GP%d IN4=GP%d\n",
           FLYWHEEL_PIN_IN3, FLYWHEEL_PIN_IN4);
    stdio_flush();
}

void flywheels_spinup(uint32_t ms) {
    // IN3 = LOW, IN4 = full PWM, wait, then cruise duty
    printf("[FW] spinup %lums — IN3=LOW IN4=full\n", (unsigned long)ms);
    stdio_flush();
    gpio_put(FLYWHEEL_PIN_IN3, 0);
    pwm_set_gpio_level(FLYWHEEL_PIN_IN4, PWM_WRAP);
    sleep_ms(ms);
    pwm_set_gpio_level(FLYWHEEL_PIN_IN4, DUTY_CRUISE);
    printf("[FW] at cruise\n"); stdio_flush();
}

void flywheels_kickstart() {
    flywheels_spinup(1200);
}

void flywheels_start() {
    gpio_put(FLYWHEEL_PIN_IN3, 0);
    pwm_set_gpio_level(FLYWHEEL_PIN_IN4, PWM_WRAP);
}

void flywheels_stop() {
    pwm_set_gpio_level(FLYWHEEL_PIN_IN4, 0);
    gpio_put(FLYWHEEL_PIN_IN3, 0);
    printf("[FW] stop\n"); stdio_flush();
}