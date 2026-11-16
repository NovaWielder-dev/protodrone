#pragma once
#include <array>

namespace Mixer {

    struct MotorCommands {
        // Resultant motor thrusts
        float fl, fr, rr, rl;

        // Helper: Iterable view
        std::array<float, 4> asArray() const {
            return {fl, fr, rr, rl};
        }
    };

    MotorCommands compute(
        float T, 
        float Up, 
        float Ur, 
        float Uy
    );
}