#include "rack_pinion.h"
#include "config_pins.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <cstdio>

// 915us  = full speed clockwise   (back, away from flywheels)
// 2441us = full speed counter-clockwise (forward, toward flywheels)
// 1500us = stop (neutral for 360 servo)
static const uint16_t FEEDER_FORWARD_US = 2441;  // toward flywheels
static const uint16_t FEEDER_BACK_US    = 915;   // away from flywheels
static const uint16_t FEEDER_STOP_US    = 1500;  // neutral — stops 360 servo
static uint32_t s_open_duration_ms = 0;  // tracks how far back the rack travelled

static uint feeder_slice;
static uint feeder_chan;

static void feeder_set_us(uint16_t us) {
    pwm_set_chan_level(feeder_slice, feeder_chan, us);
}

void feeder_init() {
    gpio_set_function(FEEDER_PIN, GPIO_FUNC_PWM);
    feeder_slice = pwm_gpio_to_slice_num(FEEDER_PIN);
    feeder_chan  = pwm_gpio_to_channel(FEEDER_PIN);
    pwm_set_clkdiv(feeder_slice, 150.0f);
    pwm_set_wrap(feeder_slice, 20000);
    pwm_set_chan_level(feeder_slice, feeder_chan, FEEDER_STOP_US);  // set level first
    pwm_set_enabled(feeder_slice, true);                            // then enable — servo sees 1500us immediately
    printf("[FEEDER] Init at position 0 (chamber closed)\n");
    stdio_flush();
}

// ── Open chamber: go back 46.7mm and stop ────────────────────────────────────
// Called after capture. Ball drops into chamber from funnel.
// Servo stays at 46.7mm position — do NOT return to 0 yet.
// Next call will be either feeder_close_chamber() or feeder_eject_ball().
void feeder_open_chamber() {
    printf("[FEEDER] Opening chamber (back 46.7mm)...\n"); stdio_flush();
    feeder_set_us(FEEDER_BACK_US);
    sleep_ms(FEEDER_OPEN_MS);
    feeder_set_us(FEEDER_STOP_US);
    s_open_duration_ms = FEEDER_OPEN_MS;  // record how far we went
    printf("[FEEDER] Chamber open — ball in chamber\n"); stdio_flush();
}

void feeder_close_chamber() {
    printf("[FEEDER] Closing chamber (returning exact distance)...\n"); stdio_flush();
    feeder_set_us(FEEDER_FORWARD_US);
    sleep_ms(s_open_duration_ms + FEEDER_CLOSE_OFFSET_MS);  // exact inverse of however far it opened
    feeder_set_us(FEEDER_STOP_US);
    s_open_duration_ms = 0;
    printf("[FEEDER] Chamber closed — at position 0\n"); stdio_flush();
}

void feeder_eject_ball() {
    // Go further back from current open position to 71mm
    printf("[FEEDER] Ejecting ball (back to 71mm)...\n"); stdio_flush();
    feeder_set_us(FEEDER_BACK_US);
    sleep_ms(FEEDER_EJECT_MS);
    feeder_set_us(FEEDER_STOP_US);

    // Return full distance: open + eject combined
    uint32_t total_ms = s_open_duration_ms + FEEDER_EJECT_MS - FEEDER_CLOSE_OFFSET_MS;
    printf("[FEEDER] Returning to 0 (total=%lums)...\n", (unsigned long)total_ms); stdio_flush();
    feeder_set_us(FEEDER_FORWARD_US);
    sleep_ms(total_ms);
    feeder_set_us(FEEDER_STOP_US);
    s_open_duration_ms = 0;
    printf("[FEEDER] At position 0\n"); stdio_flush();
}

void feeder_release() {
    feeder_set_us(FEEDER_STOP_US);
}