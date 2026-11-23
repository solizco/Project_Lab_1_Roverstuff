#include "color_sensor.h"
#include "config_pins.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <cstdio>

// ── Detection parameters (from Finn's v3, do not change without re-testing) ──
//
// FILTER_SETTLE_MS: Time for TCS3200 output to stabilize after S2/S3 change.
//                  Finn uses 40ms — old C++ used 30ms which caused noisy reads.
// LED_WARMUP_MS:   Ensure LEDs are at full brightness before first read.
// PULSE_TIMEOUT_US: Max wait per pulse edge before declaring sensor dead.
//                  Finn uses 200ms — old C++ used 50ms causing false errors.
// PULSES_TO_COUNT:  Number of HIGH→LOW transitions timed per channel read.
// SAMPLES:          Reads per channel — averaged to suppress jitter.
// CLEAR_PRESENT_MAX: CLEAR period threshold. Higher period = less light =
//                  no ball blocking the sensor. Tuned by Finn at 600us.
// DOMINANT_RATIO:  dominant_period / runner_up_period must be < this.
//                  0.80 means dominant must be at least 20% shorter (brighter).
// MIN_ABS_GAP:     runner_up - dominant must exceed this in microseconds.
//                  Rejects ambient/ceiling where all channels are flat.

static const int     SAMPLES            = 3;
static const int     LED_WARMUP_MS      = 50;
static const int     FILTER_SETTLE_MS   = 40;
static const int32_t PULSE_TIMEOUT_US   = 200000;
static const int     PULSES_TO_COUNT    = 10;
static const int32_t CLEAR_PRESENT_MAX  = 600;
static const float   DOMINANT_RATIO     = 0.80f;
static const int32_t MIN_ABS_GAP        = 125;

// CAL multipliers — all 1.0 until calibration data is collected on the field.
// To calibrate: point sensor at each ball color, note the raw period for the
// dominant channel, and scale so all three "dominant when dominant" reads match.
static const float CAL_RED   = 1.0f;
static const float CAL_GREEN = 1.0f;
static const float CAL_BLUE  = 1.0f;

void color_sensor_init() {
    gpio_init(COLOR_PIN_S2);   gpio_set_dir(COLOR_PIN_S2,  GPIO_OUT);
    gpio_init(COLOR_PIN_S3);   gpio_set_dir(COLOR_PIN_S3,  GPIO_OUT);
    gpio_init(COLOR_PIN_OUT);  gpio_set_dir(COLOR_PIN_OUT, GPIO_IN);
    gpio_init(COLOR_PIN_LED);  gpio_set_dir(COLOR_PIN_LED, GPIO_OUT);
    gpio_put(COLOR_PIN_LED, 1);   // LEDs on (ball_handler turns them off during transit)
    printf("[COLOR] Sensor init: S2=GP%d S3=GP%d OUT=GP%d LED=GP%d\n",
           COLOR_PIN_S2, COLOR_PIN_S3, COLOR_PIN_OUT, COLOR_PIN_LED);
    stdio_flush();
}

// ── Single-channel pulse train timing ────────────────────────────────────────
// Sets S2/S3, waits FILTER_SETTLE_MS, then times PULSES_TO_COUNT complete
// rising→falling transitions on OUT (matching Finn's read_pulse_train exactly).
//
// Returns total elapsed microseconds for PULSES_TO_COUNT pulses, or -1 on
// timeout. Lower return value = shorter period = higher frequency = more light
// of that color = more dominant.
//
// Finn's Python pulse counting order (reproduced here):
//   for each pulse:
//     wait for OUT to go HIGH  (rising edge)
//     wait for OUT to go LOW   (falling edge)
static int32_t read_pulse_train(int s2_val, int s3_val) {
    gpio_put(COLOR_PIN_S2, s2_val);
    gpio_put(COLOR_PIN_S3, s3_val);
    sleep_ms(FILTER_SETTLE_MS);

    uint32_t start = time_us_32();

    for (int i = 0; i < PULSES_TO_COUNT; i++) {
        // Wait for rising edge (OUT goes HIGH)
        uint32_t t = time_us_32();
        while (!gpio_get(COLOR_PIN_OUT)) {
            if ((int32_t)(time_us_32() - t) > PULSE_TIMEOUT_US) return -1;
        }
        // Wait for falling edge (OUT goes LOW)
        t = time_us_32();
        while (gpio_get(COLOR_PIN_OUT)) {
            if ((int32_t)(time_us_32() - t) > PULSE_TIMEOUT_US) return -1;
        }
    }

    return (int32_t)(time_us_32() - start);
}

// ── Averaged channel read (SAMPLES readings, mean) ───────────────────────────
static int32_t avg_channel(int s2_val, int s3_val) {
    int32_t total = 0;
    for (int i = 0; i < SAMPLES; i++) {
        int32_t val = read_pulse_train(s2_val, s3_val);
        if (val < 0) return -1;   // timeout — propagate error
        total += val;
    }
    return total / SAMPLES;
}

// ── Read all 4 channels: CLEAR, RED, GREEN, BLUE ─────────────────────────────
// Reads CLEAR before and after RGB and takes the max (Finn's exact approach).
// CLEAR uses S2=1, S3=0 (not used for color ID, only for presence detection).
typedef struct {
    int32_t red;
    int32_t green;
    int32_t blue;
    int32_t clear;  // max of clear_before and clear_after
} RawChannels;

static RawChannels read_all_channels() {
    RawChannels ch;
    gpio_put(COLOR_PIN_LED, 1);
    sleep_ms(LED_WARMUP_MS);  // let LEDs stabilize before first read

    int32_t clear_before = avg_channel(1, 0);  // CLEAR: S2=1, S3=0
    ch.red               = avg_channel(0, 0);  // RED:   S2=0, S3=0
    ch.green             = avg_channel(1, 1);  // GREEN: S2=1, S3=1
    ch.blue              = avg_channel(0, 1);  // BLUE:  S2=0, S3=1
    int32_t clear_after  = avg_channel(1, 0);  // CLEAR again

    // Take the worse (higher period = less light) of the two CLEAR reads.
    // If either shows high period, ball is not blocking the sensor.
    ch.clear = (clear_before > clear_after) ? clear_before : clear_after;
    return ch;
}

// ── Full color detection (Finn's v3 detect_color logic) ──────────────────────
BallColor color_sensor_read() {
    RawChannels ch = read_all_channels();

    // Propagate sensor errors
    if (ch.red < 0 || ch.green < 0 || ch.blue < 0 || ch.clear < 0) {
        printf("[COLOR] ERROR — sensor timeout (check wiring, LED power)\n");
        stdio_flush();
        return COLOR_ERROR;
    }

    printf("[COLOR] RAW  R=%ld G=%ld B=%ld CLR=%ld\n",
           (long)ch.red, (long)ch.green, (long)ch.blue, (long)ch.clear);
    stdio_flush();

    // Presence check — high CLEAR period means no ball is blocking the sensor.
    // Finn's threshold: CLEAR > 600us → NONE.
    if (ch.clear > CLEAR_PRESENT_MAX) {
        printf("[COLOR] NONE (CLEAR=%ld > %ld)\n",
               (long)ch.clear, (long)CLEAR_PRESENT_MAX);
        stdio_flush();
        return COLOR_NONE;
    }

    // Apply CAL multipliers (currently all 1.0 — tune on field if needed).
    // Multiplying period by CAL scales apparent brightness:
    // a CAL < 1.0 makes a channel appear brighter (shorter effective period).
    float r = (float)ch.red   * CAL_RED;
    float g = (float)ch.green * CAL_GREEN;
    float b = (float)ch.blue  * CAL_BLUE;

    // Find dominant (lowest period = highest frequency = most light).
    // Sort so sorted[0] is dominant, sorted[2] is weakest.
    // We only need the dominant and runner-up values and which color won.
    const char* dom_name;
    float dom_val, runner_val;
    BallColor dom_color;

    if (r <= g && r <= b) {
        dom_color = COLOR_RED;   dom_name = "RED";   dom_val = r;
        runner_val = (g < b) ? g : b;
    } else if (g <= r && g <= b) {
        dom_color = COLOR_GREEN; dom_name = "GREEN"; dom_val = g;
        runner_val = (r < b) ? r : b;
    } else {
        dom_color = COLOR_BLUE;  dom_name = "BLUE";  dom_val = b;
        runner_val = (r < g) ? r : g;
    }

    // Gate 1: dominant must be at least (1 - DOMINANT_RATIO) = 20% shorter
    // than the runner-up. Rejects cases where two channels are very close.
    if (dom_val / runner_val > DOMINANT_RATIO) {
        printf("[COLOR] UNCLEAR (ratio=%.2f > %.2f)\n",
               dom_val / runner_val, DOMINANT_RATIO);
        stdio_flush();
        return COLOR_UNKNOWN;
    }

    // Gate 2: absolute gap must exceed MIN_ABS_GAP.
    // Rejects flat ambient/ceiling readings where all channels are similar.
    float gap = runner_val - dom_val;
    if (gap < (float)MIN_ABS_GAP) {
        printf("[COLOR] UNCLEAR (gap=%.0f < %ld)\n",
               gap, (long)MIN_ABS_GAP);
        stdio_flush();
        return COLOR_UNKNOWN;
    }

    printf("[COLOR] DETECTED %s (dom=%.0f runner=%.0f gap=%.0f)\n",
           dom_name, dom_val, runner_val, gap);
    stdio_flush();
    return dom_color;
}

const char* color_name(BallColor c) {
    switch (c) {
        case COLOR_RED:     return "RED";
        case COLOR_GREEN:   return "GREEN";
        case COLOR_BLUE:    return "BLUE";
        case COLOR_NONE:    return "NONE";
        case COLOR_UNKNOWN: return "UNKNOWN";
        case COLOR_ERROR:   return "ERROR";
    }
    return "?";
}