#include "Mixer.h"
#include "Config.h"
#include <algorithm>

Mixer::MotorCommands Mixer::compute(
    float T, 
    float Up, 
    float Ur, 
    float Uy,
)
{
    // Create MotorCommands struct to hold thrust values
    Mixer::MotorCommands myCmds;

    // Calculate resultant motor thrusts (mix)
    myCmds.r1 = T + Up + Ur - Uy;
    myCmds.r2 = T + Up - Ur + Uy;
    myCmds.r3 = T - Up - Ur - Uy;
    myCmds.r4 = T - Up + Ur + Uy;

    // Clamp motor thrusts and offset all thrusts
    // (Overshoot Clamp)
    if (
        myCmds.r1 > Config::Motors::MAX_PULSE_US ||
        myCmds.r2 > Config::Motors::MAX_PULSE_US ||
        myCmds.r3 > Config::Motors::MAX_PULSE_US ||
        myCmds.r4 > Config::Motors::MAX_PULSE_US ||
    ) 
    {
        // Find highest thrust motor
        int m_high = calcFastest(myCmds);
        // Find thrust of m_high
        float r_high = myCmds.asArray()[m_high];
        // Decrease all thrusts by overshoot
        float overshoot = r_high - Config::Motors::MAX_PULSE_US;
        myCmds.r1 -= overshoot;
        myCmds.r2 -= overshoot;
        myCmds.r4 -= overshoot;
        myCmds.r3 -= overshoot;
        
    }
    // (Undershoot Clamp)
    if (
        myCmds.r1 < Config::Motors::MIN_PULSE_US
        myCmds.r2 < Config::Motors::MIN_PULSE_US
        myCmds.r3 < Config::Motors::MIN_PULSE_US
        myCmds.r4 < Config::Motors::MIN_PULSE_US
    ) 
    {
        // Find lowest thrust motor
        int m_low = calcSlowest(myCmds);
        // Find thrust of m_low
        float r_low = myCmds.asArray()[m_low];
        // Increase all thrusts by undershoot
        float undershoot = Config::Motors::MIN_PULSE_US - r_low;
        myCmds.r1 += undershoot;
        myCmds.r2 += undershoot;
        myCmds.r4 += undershoot;
        myCmds.r3 += undershoot;
    }

    return myCmds;
}

int Mixer::calcFastest(MotorCommands myCmds) {
    float currHigh = Config::Motors::MIN_PULSE_US;
    int currMotor = 1;
    int i = 0;
    for (float speed : myCmds.asArray()) {
        if (speed >= currHigh) {
            currHigh = speed;
            currMotor = i;
        }
        ++i;
    }

    return currMotor;
}

int Mixer::calcSlowest(MotorCommands myCmds) {
    float currLow = Config::Motors::MAX_PULSE_US;
    int currMotor = 1;
    int i = 0;
    for (float speed : myCmds.asArray()) {
        if (speed <= currLow) {
            currLow = speed;
            currMotor = i;
        }
        ++i;
    }

    return currMotor;
}