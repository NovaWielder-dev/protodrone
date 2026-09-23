#include "Mixer.h"

Mixer::MotorCommands Mixer::compute(
    float T, 
    float Up, 
    float Ur, 
    float Uy
)
{
    // Create MotorCommands struct to hold thrust values
    Mixer::MotorCommands myCommands;
    // Calculate resultant motor thrusts
    myCommands.m1 = T + Up + Ur - Uy;
    myCommands.m2 = T + Up - Ur + Uy;
    myCommands.m3 = T - Up - Ur + Uy;
    myCommands.m4 = T - Up + Ur - Uy;

    return myCommands;
}