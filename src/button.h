#pragma once
#include <stdbool.h>

void button_init();
// Returns true once per press (debounced, non-blocking)
bool button_pressed_edge();