#pragma once
#include "hardware/spi.h"
#include "hardware/i2c.h"

// Single source of truth for all GPIO assignments (Raspberry Pi Pico 2).
// "pin" = physical header pin; "slice" = RP2350 PWM slice/channel.
// No two PWM outputs that need different frequencies share a slice.

// ── Collector arm servos (MG995, 50 Hz, share PWM slice 0) ──────────────────
#define ARM_PIN_B         0    // pin 1,  slice 0A
#define ARM_PIN_A         1    // pin 2,  slice 0B

// ── Wheel encoders (single-channel, falling-edge IRQ) ───────────────────────
#define ENC_PIN_LEFT      2    // pin 4
#define ENC_PIN_RIGHT     3    // pin 5

// ── ToF sensors (shared I2C0 + one XSHUT line per sensor) ───────────────────
#define TOF_I2C           i2c0
#define TOF_PIN_SDA       4    // pin 6
#define TOF_PIN_SCL       5    // pin 7
#define TOF_PIN_XSHUT_FL  6    // pin 9   (purple) — front-left  VL53L0X
#define TOF_PIN_XSHUT_FM  7    // pin 10  (gray)   — front-mid   VL53L1X
#define TOF_PIN_XSHUT_FR  8    // pin 11  (blue)   — front-right VL53L0X

// ── Mode switches (active-low, internal pull-ups) ────────────────────────────
#define corner_orientation_switch 9    // pin 12 — open = LEFT corner, closed = RIGHT
#define mode_switch               14   // pin 19 — open = KICKER, closed = GOALIE

// ── Color sensor (TCS3200) ───────────────────────────────────────────────────
// S0 and S1 are hard-wired on the board (S0 = 3V3, S1 = GND → 20% frequency
// scaling), so they do not use Pico GPIOs.
#define COLOR_PIN_LED     10   // pin 14 (white)
#define COLOR_PIN_S2      11   // pin 15 (yellow)
#define COLOR_PIN_S3      12   // pin 16 (orange)
#define COLOR_PIN_OUT     13   // pin 17 (red)

// ── Feeder (continuous-rotation servo, 50 Hz) ───────────────────────────────
#define FEEDER_PIN        15   // pin 20, slice 7B

// ── Camera (Arducam Mega on SPI0) ────────────────────────────────────────────
#define CAM_SPI           spi0
#define CAM_PIN_MISO      16   // pin 21
#define CAM_PIN_CS        17   // pin 22
#define CAM_PIN_SCK       18   // pin 24
#define CAM_PIN_MOSI      19   // pin 25
#define CAM_PIN_RST       -1   // not connected

// ── Flywheel (H-bridge) ──────────────────────────────────────────────────────
// IN3 is a plain GPIO held LOW (forward). Only IN4 needs PWM for speed.
#define FLYWHEEL_PIN_IN3  20   // pin 26 (brown)  — plain GPIO, held LOW
#define FLYWHEEL_PIN_IN4  21   // pin 27 (purple) — PWM speed, slice 2B

// ── Drive motors (H-bridge) ──────────────────────────────────────────────────
// Forward: IN_a = PWM, IN_b = LOW | Reverse: IN_a = LOW, IN_b = PWM | Stop: both LOW
#define MOTOR_L_PIN_IN4   22   // pin 29 (green),  slice 3A
#define MOTOR_R_PIN_IN1   26   // pin 31 (gray),   slice 5A
#define MOTOR_L_PIN_IN3   27   // pin 32 (blue),   slice 5B
#define MOTOR_R_PIN_IN2   28   // pin 34 (purple), slice 6A

// ── Onboard LED (Pico 2; on a Pico 2 W the LED is on the wireless chip) ──────
#define LED_PIN           25
