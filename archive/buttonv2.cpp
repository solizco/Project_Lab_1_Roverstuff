#include "button.h"
#include "config_pins.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

static const uint32_t DEBOUNCE_US = 20000;

static bool     last_raw_state = false;
static bool     stable_state   = false;
static uint32_t last_change_us = 0;

void button_init() {
    gpio_init(BTN_PIN);
    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_pull_up(BTN_PIN);
    last_raw_state = !gpio_get(BTN_PIN);
    stable_state   = last_raw_state;
    last_change_us = time_us_32();
}

bool button_pressed_edge() {
    bool raw = !gpio_get(BTN_PIN);  // active-low
    uint32_t now = time_us_32();

    if (raw != last_raw_state) {
        last_raw_state = raw;
        last_change_us = now;
    }
    if ((now - last_change_us) >= DEBOUNCE_US) {
        if (stable_state != last_raw_state) {
            stable_state = last_raw_state;
            if (stable_state) return true;
        }
    }
    return false;
}