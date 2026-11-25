#pragma once
#include "Config.h"

namespace Receiver {

    struct ControlTargets {
        bool armed = false;
        float throttle = Config::Motors::MIN_PULSE_US;
        float pitch = 0.0f;
        float roll = 0.0f;
        float yawRate = 0.0f;
    };

    void init();
    void update();
    ControlTargets getTargets();
    bool isConnected();
}