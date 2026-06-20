#pragma once
#include <cstdint>
#include "status_codes.h"

void initStatusLed();
void setStatusLed(LedCode code);         // thread-safe: stores code atomically
LedCode getStatusLed();                  // read current code
void ledTask(void* param);               // FreeRTOS task — renders blink/pulse patterns

// Colour/pattern table entry
struct LedPattern {
    uint8_t r, g, b;
    uint16_t blink_ms;  // 0 = solid, non-zero = toggle at this interval
};

const LedPattern& getPatternForCode(LedCode code);
