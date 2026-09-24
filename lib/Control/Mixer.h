#pragma once
#include <array>

namespace Mixer {

    struct MotorCommands {
        // Resultant motor thrusts
        float r1, r2, r3, r4;

        // Helper: Iterable view
        std::array<float, 4> asArray() const {
            return {r1, r2, r3, r4};
        }
    };

    MotorCommands compute(
        float T, 
        float Up, 
        float Ur, 
        float Uy
    );

    int calcFastest(MotorCommands myCmds);
    int calcSlowest(MotorCommands myCmds);
}