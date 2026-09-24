#include "Mixer.h"
#include "Config.h"
#include <algorithm>

Mixer::MotorCommands Mixer::compute(
    float T, 
    float Up, 
    float Ur, 
    float Uy
)
{
    // Create MotorCommands struct to hold thrust values
    Mixer::MotorCommands myCmds;

    // Calculate resultant motor thrusts (mix)
    myCmds.r1 = T + Up + Ur - Uy;
    myCmds.r2 = T + Up - Ur + Uy;
    myCmds.r3 = T - Up - Ur - Uy;
    myCmds.r4 = T - Up + Ur + Uy;

    /* === Clamp motor thrusts and offset all thrusts === */

    // 1. (Overshoot Clamp)
    float r_high = std::max({myCmds.r1, myCmds.r2, myCmds.r3, myCmds.r4});
    // Check for overshoot
    if (r_high > Config::Motors::MAX_PULSE_US) 
    {
        // Decrease all thrusts by overshoot
        float overshoot = r_high - Config::Motors::MAX_PULSE_US;
        myCmds.r1 -= overshoot;
        myCmds.r2 -= overshoot;
        myCmds.r4 -= overshoot;
        myCmds.r3 -= overshoot;
        
    }
    // 2. Undershoot Clamp
    float r_low = std::min({myCmds.r1, myCmds.r2, myCmds.r3, myCmds.r4});
    // Check for undershoot
    if (r_low < Config::Motors::MIN_PULSE_US) 
    {
        // Increase all thrusts by undershoot
        float undershoot = Config::Motors::MIN_PULSE_US - r_low;
        myCmds.r1 += undershoot;
        myCmds.r2 += undershoot;
        myCmds.r4 += undershoot;
        myCmds.r3 += undershoot;
    }

    /* === Bandwidth Clamp === */
    myCmds.r1 = std::clamp
    (
        myCmds.r1, 
        Config::Motors::MIN_PULSE_US, 
        Config::Motors::MAX_PULSE_US
    );
    myCmds.r2 = std::clamp
    (
        myCmds.r2, 
        Config::Motors::MIN_PULSE_US, 
        Config::Motors::MAX_PULSE_US
    );
    myCmds.r3 = std::clamp
    (
        myCmds.r3, 
        Config::Motors::MIN_PULSE_US, 
        Config::Motors::MAX_PULSE_US
    );
    myCmds.r4 = std::clamp
    (
        myCmds.r4, 
        Config::Motors::MIN_PULSE_US, 
        Config::Motors::MAX_PULSE_US
    );

    return myCmds;
}