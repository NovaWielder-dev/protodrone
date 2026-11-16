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
    myCmds.fl = T + Up + Ur - Uy;
    myCmds.fr = T + Up - Ur + Uy;
    myCmds.rr = T - Up - Ur - Uy;
    myCmds.rl = T - Up + Ur + Uy;

    /* === Clamp motor thrusts and offset all thrusts === */

    // 1. (Overshoot Clamp)
    float r_high = std::max({myCmds.fl, myCmds.fr, myCmds.rr, myCmds.rl});
    // Check for overshoot
    if (r_high > Config::Motors::MAX_PULSE_US) 
    {
        // Decrease all thrusts by overshoot
        float overshoot = r_high - Config::Motors::MAX_PULSE_US;
        myCmds.fl -= overshoot;
        myCmds.fr -= overshoot;
        myCmds.rr -= overshoot;
        myCmds.rl -= overshoot;
        
    }
    // 2. Undershoot Clamp
    float r_low = std::min({myCmds.fl, myCmds.fr, myCmds.rr, myCmds.rl});
    // Check for undershoot
    if (r_low < Config::Motors::MIN_PULSE_US) 
    {
        // Increase all thrusts by undershoot
        float undershoot = Config::Motors::MIN_PULSE_US - r_low;
        myCmds.fl += undershoot;
        myCmds.fr += undershoot;
        myCmds.rr += undershoot;
        myCmds.rl += undershoot;
    }

    /* === Bandwidth Clamp === */
    myCmds.fl = std::clamp
    (
        myCmds.fl, 
        Config::Motors::MIN_PULSE_US, 
        Config::Motors::MAX_PULSE_US
    );
    myCmds.fr = std::clamp
    (
        myCmds.fr, 
        Config::Motors::MIN_PULSE_US, 
        Config::Motors::MAX_PULSE_US
    );
    myCmds.rr = std::clamp
    (
        myCmds.rr, 
        Config::Motors::MIN_PULSE_US, 
        Config::Motors::MAX_PULSE_US
    );
    myCmds.rl = std::clamp
    (
        myCmds.rl,
        Config::Motors::MIN_PULSE_US, 
        Config::Motors::MAX_PULSE_US
    );

    return myCmds;
}