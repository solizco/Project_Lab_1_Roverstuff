#include "ball_handler.h"
#include "color_sensor.h"
#include "flywheel_motors.h"
#include "rack_pinion.h"
#include "config_pins.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <cstdio>

// ── Sequence parameters ───────────────────────────────────────────────────────
// CONFIRM_READS: consecutive matching reads required to lock in a color.
// EMPTY_LIMIT:   consecutive NONE reads before identify() gives up.
// LOOP_DELAY_MS: pause between color reads (sensor itself takes ~1.5s per call).
// BALL_DWELL_MS: time after feeder fires before stopping flywheels —
//               gives ball time to fully clear the flywheel gap at cruise speed.
static const int     CONFIRM_READS  = 4;
static const int     EMPTY_LIMIT    = 8;
static const int     LOOP_DELAY_MS  = 200;
static const uint32_t BALL_DWELL_MS = 350;
// MAX_READS:   total reads before identify() gives up on an unclear ball.
// ERROR_LIMIT: consecutive sensor timeouts before identify() reports an error.
static const int     MAX_READS      = 20;
static const int     ERROR_LIMIT    = 5;

// Stores the color detected by ball_handler_identify() so ball_handler_fire()
// can act on it after the rover returns to origin.
static BallColor s_last_color = COLOR_NONE;

void ball_handler_init() {
    flywheel_init();
    feeder_init();
    color_sensor_init();
    printf("[BH] Ball handler subsystems initialized\n");
    stdio_flush();
}

BallColor ball_handler_get_last_color() {
    return s_last_color;
}

// ── Identify ──────────────────────────────────────────────────────────────────
// Runs the color-detect streak loop. Does NOT fire the feeder or run flywheels.
// The ball sits in the chamber during this step.
//
// Exits when any of:
//   a) CONFIRM_READS consecutive matching reads → color confirmed, stored
//   b) EMPTY_LIMIT consecutive NONE reads      → no ball found, COLOR_NONE stored
//   c) MAX_READS total reads without a result   → COLOR_UNKNOWN stored
//   d) ERROR_LIMIT consecutive sensor timeouts  → COLOR_ERROR stored
//
// On confirmed color: sensor LED is turned off so the ball doesn't get stray
// reads from the sensor while the rover transits back to origin.
// ball_handler_fire() turns the LED back on before executing.
BallColor ball_handler_identify() {
    printf("[BH] ====== ball_handler_identify() start ======\n");
    stdio_flush();

    s_last_color = COLOR_NONE;

    BallColor last_color  = COLOR_NONE;
    int       streak      = 0;
    int       empty_count = 0;
    int       error_count = 0;
    int       reads       = 0;

    while (true) {
        if (++reads > MAX_READS) {
            printf("[BH] No confirmed color after %d reads — giving up\n", MAX_READS);
            stdio_flush();
            s_last_color = COLOR_UNKNOWN;
            break;
        }

        BallColor color = color_sensor_read();

        // ── Sensor failure ────────────────────────────────────────────────────
        if (color == COLOR_ERROR) {
            if (++error_count >= ERROR_LIMIT) {
                printf("[BH] SENSOR ERROR x%d — giving up (check wiring)\n", ERROR_LIMIT);
                stdio_flush();
                s_last_color = COLOR_ERROR;
                break;
            }
            printf("[BH] SENSOR ERROR — check wiring. Retrying in 1s\n");
            stdio_flush();
            sleep_ms(1000);
            continue;
        }
        error_count = 0;

        // ── No ball present ───────────────────────────────────────────────────
        if (color == COLOR_NONE) {
            empty_count++;
            printf("[BH] NONE  empty=%d/%d\n", empty_count, EMPTY_LIMIT);
            stdio_flush();
            if (empty_count >= EMPTY_LIMIT) {
                printf("[BH] No ball detected after %d cycles — exiting\n",
                       EMPTY_LIMIT);
                stdio_flush();
                s_last_color = COLOR_NONE;
                break;
            }
            sleep_ms(LOOP_DELAY_MS);
            continue;
        }

        // A ball is present — reset empty counter
        empty_count = 0;

        // ── Ambiguous reading ─────────────────────────────────────────────────
        if (color == COLOR_UNKNOWN) {
            printf("[BH] UNCLEAR  streak=%d (holding)\n", streak);
            stdio_flush();
            sleep_ms(LOOP_DELAY_MS);
            continue;
        }

        // ── Confirmed color (RED / GREEN / BLUE) ──────────────────────────────
        if (color == last_color) {
            streak++;
        } else {
            streak     = 1;
            last_color = color;
        }

        printf("[BH] %s  streak=%d/%d\n",
               color_name(color), streak, CONFIRM_READS);
        stdio_flush();

        if (streak >= CONFIRM_READS) {
            printf("[BH] *** CONFIRMED %s — storing, disabling sensor LED for transit ***\n",
                   color_name(color));
            stdio_flush();
            s_last_color = color;
            gpio_put(COLOR_PIN_LED, 0);  // kill sensor LED — no stray reads during transit
            break;
        }

        sleep_ms(LOOP_DELAY_MS);
    }

    printf("[BH] ====== ball_handler_identify() done — stored: %s ======\n\n",
           color_name(s_last_color));
    stdio_flush();
    return s_last_color;
}

// ── Fire ──────────────────────────────────────────────────────────────────────
// Called from STATE_FIRE after rover has returned to origin.
// Restores sensor LED, then acts on the color stored by identify().
//
// RED          → feeder_close_chamber() pushes ball into flywheels → opponent goal
// GREEN / BLUE → feeder_eject_ball() drops ball at own goal → own score
//
// If no confirmed color is stored, closes chamber and returns without firing.
void ball_handler_fire() {
    printf("[BH] ====== ball_handler_fire() start — color: %s ======\n",
           color_name(s_last_color));
    stdio_flush();

    gpio_put(COLOR_PIN_LED, 1);

    if (s_last_color == COLOR_NONE || s_last_color == COLOR_UNKNOWN ||
        s_last_color == COLOR_ERROR) {
        printf("[BH] No confirmed ball — closing chamber and skipping fire\n");
        stdio_flush();
        feeder_close_chamber();
        return;
    }

    if (s_last_color == COLOR_RED) {
        printf("[BH] RED — spinning up 6500ms then firing\n"); stdio_flush();
        flywheels_spinup(6500);          // proven working sequence, extended spin-up
        feeder_close_chamber();
        sleep_ms(BALL_DWELL_MS);
        flywheels_stop();
    } else {
        printf("[BH] %s → ejecting at own goal\n", color_name(s_last_color)); stdio_flush();
        feeder_eject_ball();
    }

    printf("[BH] ====== ball_handler_fire() done ======\n\n");
    stdio_flush();
    s_last_color = COLOR_NONE;
}