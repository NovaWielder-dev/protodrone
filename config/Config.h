#pragma once
#include <cstdint>

namespace Config::Motors {
    // PWM pulse values (us) for maximum and minimum allowed thrust
    constexpr float MIN_PULSE_US = 1000; // Zero thrust
    constexpr float MAX_PULSE_US = 1750; // 75% of motor max 

    // ESC Hardware GPIO Mapping
    constexpr int PIN_FRONT_LEFT = 26; // esc2
    constexpr int PIN_FRONT_RIGHT = 25; //esc1
    constexpr int PIN_REAR_RIGHT = 32; // esc3
    constexpr int PIN_REAR_LEFT = 33; // esc4
}