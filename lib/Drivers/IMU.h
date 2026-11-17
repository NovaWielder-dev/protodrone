#pragma once

namespace IMU {

    struct Attitude {
        float pitch;
        float roll;
        float yawRate; // Physics: since gravity has no effect on yaw
    };

    bool init();
    void calibrate();
    Attitude update(float dt);

}
