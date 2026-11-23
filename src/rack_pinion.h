#pragma once

// 360 servo rack and pinion — position controlled by timed pulses.
// Position 0 = closed (ball cannot enter chamber).
//
// Tuning constants — adjust MS values until distances match physical measurements:
//   FEEDER_OPEN_MS:   time to travel 46.7mm back (open chamber, ball drops in)
//   FEEDER_EJECT_MS:  extra time to travel on to 71mm back (eject green/blue out rear)
//   FEEDER_CLOSE_OFFSET_MS: return-time adjustment so the rack lands at 0
//
// Pulse widths for the 360° servo are set in rack_pinion.cpp:
// FEEDER_FORWARD_US: pulse width for forward direction (toward flywheels)
// FEEDER_BACK_US:    pulse width for backward direction (away from flywheels)

#define FEEDER_OPEN_MS    600   // 46.7mm back — tune this first
#define FEEDER_EJECT_MS   800   // 71mm back from open position — tune after OPEN
#define FEEDER_CLOSE_OFFSET_MS  100  // added when closing, subtracted after an eject


void feeder_init();
void feeder_open_chamber();   // go back 46.7mm, let ball fall in, stop
void feeder_close_chamber();  // return to 0 from 46.7mm (RED path)
void feeder_eject_ball();     // go back to 71mm then return to 0 (GREEN/BLUE path)
void feeder_release();        // kill PWM signal