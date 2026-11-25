#pragma once
#include <cstdint>
#include "Secrets.h"

namespace Config::Motors {
    // PWM pulse values (us) for maximum and minimum allowed thrust
    constexpr float MIN_PULSE_US = 1000; // Zero thrust
    constexpr float MAX_PULSE_US = 1750; // 75% of motor max 

    // ESC Hardware GPIO Mapping
    constexpr int PIN_FRONT_LEFT = 26; // esc2
    constexpr int PIN_FRONT_RIGHT = 25; //esc1
    constexpr int PIN_REAR_RIGHT = 32; // esc3
    constexpr int PIN_REAR_LEFT = 33; // esc4
}

namespace Config::IMU {
    constexpr int GYRO_SCL = 22;
    constexpr int GYRO_SDA = 21;
}

namespace Config::WiFi {
    constexpr uint32_t FAILSAFE_TIMEOUT_MS = 300;
    constexpr const char* SSID = Config::Secrets::WIFI_SSID;
    constexpr const char* PASSWORD = Config::Secrets::WIFI_PASSWORD;
    constexpr int UDP_PORT = 4210;
    constexpr float MAX_ANGLE_DEG = 20.0f;
    constexpr float MAX_YAW_RATE_DPS = 60.0f;
}