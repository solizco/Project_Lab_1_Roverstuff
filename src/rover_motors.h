#pragma once
void motors_init();
void motors_set(float left, float right);  // -1.0 (full reverse) to +1.0 (full forward)
void motors_stop();
void motors_dump_state(const char* context);  // prints full PWM slice state for all 4 pins