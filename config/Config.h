#pragma once
#include <cstdint>

namespace Config::Motors {
    // PWM pulse values (us) for maximum and minimum allowed thrust
    constexpr float MIN_PULSE_US = 1000; // Zero thrust
    constexpr float MAX_PULSE_US = 1750; // 75% of motor max 
}