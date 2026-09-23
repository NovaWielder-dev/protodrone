#ifndef MIXER_H
#define MIXER_H

namespace Mixer {

    struct MotorCommands {
        float m1, m2, m3, m4;
    };

    MotorCommands compute(
        float throttle, 
        float pitch, 
        float roll, 
        float yaw
    );
}

#endif