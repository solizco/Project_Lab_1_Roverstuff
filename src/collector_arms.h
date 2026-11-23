#pragma once

// Both arms move together. Servo B gets (180 - angle) to
// compensate for its reversed physical mounting.
#define ARMS_ROM_MIN  5
#define ARMS_ROM_MAX  126       // degrees — physical limit of mechanism
#define ARM_A_TRIM_TOP_DEG  -14   // trim at ARMS_ROM_MAX (top) — already tuned
#define ARM_A_TRIM_BOT_DEG    12   // trim at ARMS_ROM_MIN (bottom) — tune this
#define ARM_DOWN_DELAY_US  20000 // increase to slow down, decrease to speed up
#define ARM_UP_DELAY_US    18000   // keep same as down for matching speed

void arms_init();
void arms_sweep(int start_deg, int end_deg);  // blocking, ARM_DOWN/UP_DELAY_US per degree
void arms_set(int angle_deg);
void arms_release();  // removes PWM signal, servos go limp
int arms_get_angle();