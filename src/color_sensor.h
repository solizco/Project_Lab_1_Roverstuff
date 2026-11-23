#pragma once

// TCS3200 color sensor driver — ported from Finn's MicroPython v3.
//
// S0/S1 are HARDWIRED on the breakout board (S0=3V3, S1=GND → 20% scaling).
// Pico drives only S2, S3, LED, and reads OUT.
//
// Detection algorithm:
//   1. Read CLEAR channel before and after RGB to check ball presence.
//      If CLEAR > CLEAR_PRESENT_MAX (600us) → no ball → COLOR_NONE.
//   2. Read RED (S2=0,S3=0), GREEN (S2=1,S3=1), BLUE (S2=0,S3=1).
//      Lower period = more light of that color = dominant.
//   3. dominant / runner_up must be < DOMINANT_RATIO (0.80) — i.e. dominant
//      must be at least 20% lower than runner-up.
//   4. runner_up - dominant must be > MIN_ABS_GAP (125us) to reject flat
//      ambient/ceiling readings where all channels look similar.
//
// color_sensor_read() blocks ~1.5s (3 samples × 4 channels × settle + pulses).

typedef enum {
    COLOR_RED,      // -2 pts → feeder FORWARD (opponent goal)
    COLOR_GREEN,    // +2 pts → feeder BACK    (own side)
    COLOR_BLUE,     // +1 pt  → feeder BACK    (own side)
    COLOR_NONE,     // CLEAR channel high — no ball present
    COLOR_UNKNOWN,  // ball present but no channel clearly dominant
    COLOR_ERROR     // sensor timeout — check wiring or LED power
} BallColor;

void      color_sensor_init();
BallColor color_sensor_read();   // blocking ~1.5s per call
const char* color_name(BallColor c);