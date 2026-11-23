#pragma once
#include <cstdint>
#include "pico/stdlib.h"

typedef bool    boolean;
typedef uint8_t byte;

static inline uint32_t millis() {
    return to_ms_since_boot(get_absolute_time());
}

static inline uint32_t micros() {
    return time_us_32();
}

static inline void delay(uint32_t ms) {
    sleep_ms(ms);
}

static inline void delayMicroseconds(uint32_t us) {
    sleep_us(us);
}