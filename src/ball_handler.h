#pragma once
#include "color_sensor.h"

// Ball handler — integrates color sensor, flywheel, and feeder servo.
//
// Flow:
//   STATE_CAPTURE → arms sweep down/up → feeder_open_chamber() → ball drops in
//   STATE_SORT    → ball_handler_identify()
//                     Runs color-detect loop until CONFIRM_READS consecutive
//                     matching reads. Stores result. Turns off sensor LED.
//                     Blocks until confirmed, EMPTY_LIMIT NONE reads,
//                     MAX_READS total reads, or ERROR_LIMIT sensor errors.
//   STATE_RETURN  → nav_return_to_origin() — ball rides in the open chamber
//   STATE_FIRE    → ball_handler_fire()
//                     Restores sensor LED.
//                     Fires feeder based on stored color:
//                       RED          → spin flywheel up (6.5 s), then
//                                      feeder_close_chamber() pushes the ball
//                                      into it; wait BALL_DWELL_MS; stop flywheel
//                       GREEN / BLUE → feeder_eject_ball() (drop at own goal)

void      ball_handler_init();
BallColor ball_handler_identify();        // blocking ~1.5s per read × confirms
void      ball_handler_fire();            // blocking: ~7.5s for RED, ~2s for GREEN/BLUE
BallColor ball_handler_get_last_color();  // returns what identify() last stored