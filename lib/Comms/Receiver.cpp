#include "Receiver.h"
#include "Config.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <cstring>
#include <cmath>
#include <algorithm>

namespace {

    WiFiUDP udp;
    Receiver::ControlTargets currTargets{};
    uint32_t lastValidPacketMs = 0;
    bool connected = false;

    constexpr uint8_t MAGIC_1 = 0xAA;
    constexpr uint8_t MAGIC_2 = 0x55;
    constexpr int PACKET_SIZE = 20;
    constexpr float FULL_MIN_US = 1000.0f;
    constexpr float FULL_MAX_US = 2000.0f;
}

void Receiver::init() {
    /* Creates a WiFi network for the ground_station to join */
    WiFi.softAP(
        Config::WiFi::SSID, 
        Config::WiFi::PASSWORD
    );
    udp.begin(Config::WiFi::UDP_PORT);

    Serial.println("[+] Created drone WiFi network!");
    Serial.println("\n>>> WiFi IP address: ");
    Serial.println(WiFi.softAPIP());
}

void Receiver::update() {
    /* Continually reads packets from WiFi port into currTargets struct*/

    // Drain loop while packet length != 0
    while (int packetSize = udp.parsePacket()) {

        // Discard packets with invalid sizes
        if (packetSize != PACKET_SIZE) {
            uint8_t sink[64];
            while (udp.available() > 0) {
                udp.read(sink, 64);
            }
            continue;
        }

        // Read and validate incoming packet
        uint8_t buff[PACKET_SIZE];
        if (udp.read(buff, PACKET_SIZE) != PACKET_SIZE) {
            continue;
        }

        // Validate magic bytes
        if (buff[0] != MAGIC_1 || buff[1] != MAGIC_2) {
            continue;
        }
        
        // Validate checksum byte
        uint8_t expected = 0;
        for (int i = 0; i < PACKET_SIZE - 1; ++i) {
            expected ^= buff[i];
        }
        if (buff[PACKET_SIZE - 1] != expected) {
            continue;
        }

        // Copy target values from buffer to temp floats
        float tempThrottle = 0.0f;
        float tempPitch = 0.0f;
        float tempRoll = 0.0f;
        float tempYawRate = 0.0f;
        std::memcpy(&tempThrottle, &buff[3], sizeof(float));
        std::memcpy(&tempPitch, &buff[7], sizeof(float));
        std::memcpy(&tempRoll, &buff[11], sizeof(float));
        std::memcpy(&tempYawRate, &buff[15], sizeof(float));

        // Reject NaN/Inf values from ground_station
        if (
            !std::isfinite(tempThrottle) ||
            !std::isfinite(tempPitch) ||
            !std::isfinite(tempRoll) ||
            !std::isfinite(tempYawRate)
        )
        {
            continue;
        }

        // Scale throttle from percentage to microseconds
        tempThrottle = FULL_MIN_US + (tempThrottle * (FULL_MAX_US - FULL_MIN_US));

        // Assign clamped floats to currTargets
        currTargets.armed = (buff[2] != 0);
        currTargets.throttle = std::clamp(
            tempThrottle, 
            Config::Motors::MIN_PULSE_US, 
            Config::Motors::MAX_PULSE_US
        );
        currTargets.pitch = std::clamp(
            tempPitch,
            - Config::WiFi::MAX_ANGLE_DEG,
            Config::WiFi::MAX_ANGLE_DEG
        );
        currTargets.roll = std::clamp(
            tempRoll,
            - Config::WiFi::MAX_ANGLE_DEG,
            Config::WiFi::MAX_ANGLE_DEG
        );
        currTargets.yawRate = std::clamp(
            tempYawRate,
            - Config::WiFi::MAX_YAW_RATE_DPS,
            Config::WiFi::MAX_YAW_RATE_DPS
        );

        lastValidPacketMs = millis();
        connected = true;
    }

    // Watchdog heartbeat check when zero packets in buffer (packet length == 0)
    if (connected && (millis() - lastValidPacketMs) > Config::WiFi::FAILSAFE_TIMEOUT_MS) {
        connected = false;
        currTargets = Receiver::ControlTargets{};
    }
}

Receiver::ControlTargets Receiver::getTargets() {
    /* currTargets getter function */
    return currTargets;
}

bool Receiver::isConnected() {
    /* connected getter function */
    return connected;
}