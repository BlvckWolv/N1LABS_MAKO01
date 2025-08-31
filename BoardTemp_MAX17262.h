#pragma once
#include <stdint.h>

// Simple board temperature helper for Portenta H7 using MAX17262 (I2C 0x36).
// Non-blocking 1 Hz poll; returns Fahrenheit as int, or -999 if unavailable.
//
// API
// ----
// void BoardTemp_begin();
// void BoardTemp_poll_1Hz();
// int  BoardTemp_getF();
//
// Place this file and the matching BoardTemp_MAX17262.cpp in the same sketch folder
// as your M7.ino. Keep `#include "BoardTemp_MAX17262.h"` in M7.ino.

#ifdef __cplusplus
extern "C" {
#endif

void BoardTemp_begin(void);
void BoardTemp_poll_1Hz(void);
int  BoardTemp_getF(void);

#ifdef __cplusplus
} // extern "C"
#endif
