#pragma once
#include "Mixer.h"

namespace Motors {
    void init();
    void stop();
    void write(const Mixer::MotorCommands& cmds);
}