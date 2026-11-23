#pragma once
#include <cstdint>
// Flywheel driver — H-bridge
//   FLYWHEEL_PIN_IN3 (GP20) = direction, held LOW permanently (forward)
//   FLYWHEEL_PIN_IN4 (GP21) = PWM speed control (20 kHz)
//
// flywheels_spinup(ms) runs the wheel at full duty for `ms` so it reaches
// speed before the feeder pushes the ball in, then holds the cruise duty.

void flywheel_init();
void flywheels_spinup(uint32_t ms);  // full power for ms then drop to cruise
void flywheels_kickstart();          // spinup(1200)
void flywheels_start();              // full power, no wait
void flywheels_stop();