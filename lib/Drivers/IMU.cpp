#include "IMU.h"
#include "Config.h"
#include <Wire.h>
#include <Arduino.h>

// Read helpers and measured data across ticks
namespace {

    // Current normalised angular velocity (deg/s)
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;
    // Current normalised linear acceleration (g)
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;

    // Current values of roll and pitch (yaw calculated on demand)
    float estRoll = 0.0f;
    float estPitch = 0.0f;

    float pitchOffset = 0.0f;
    float rollOffset = 0.0f;
    float yawOffset = 0.0f;

    void readGyro() {

        Wire.beginTransmission(0x68); // 
        Wire.write(0x43); //
        Wire.endTransmission(false); //

        // Check
        if (Wire.requestFrom(0x68, 6) == 6) {
            int16_t rawX = (Wire.read() << 8 | Wire.read());
            int16_t rawY = (Wire.read() << 8 | Wire.read());
            int16_t rawZ = (Wire.read() << 8 | Wire.read());

            gx = rawX / 131.0f;
            gy = rawY / 131.0f;
            gz = rawZ / 131.0f;
        }
    }

    void readAccel() {

        Wire.beginTransmission(0x68);
        Wire.write(0x3B);
        Wire.endTransmission(false);

        if (Wire.requestFrom(0x68, 6) == 6) {
            int16_t rawX = (Wire.read() << 8 | Wire.read());
            int16_t rawY = (Wire.read() << 8 | Wire.read());
            int16_t rawZ = (Wire.read() << 8 | Wire.read());

            ax = rawX / 16384.0f;
            ay = rawY / 16384.0f;
            az = rawZ / 16384.0f;
        }
    }
}

bool IMU::init() {
    Wire.begin(Config::IMU::GYRO_SDA, Config::IMU::GYRO_SCL, 400000);

    Wire.beginTransmission(0x68);
    Wire.write(0x6B); // Select PWR_MGMT_1 register in MPU6050
    Wire.write(0x00); // Clear sleep bit and wake up

    if (Wire.endTransmission() != 0) {
        Serial.println("[!] ERROR: MPU6050 not found! Check SDA/SCL wiring.");
        return false;
    }

    return true;
}

void IMU::calibrate() {
    
    float totalPitch = 0.0f;
    float totalRoll = 0.0f;
    float totalYaw = 0.0f;

    /* ===== CALIBRATE GYRO ===== */
    for (int i = 0; i < 200; i++) {
        readGyro();
        totalRoll += gx;
        totalPitch += gy;
        totalYaw += gz;
        delay(2);
    }
    
    rollOffset = totalRoll / 200.0f;
    pitchOffset = totalPitch / 200.0f;
    yawOffset = totalYaw / 200.0f;

    /* ===== INITIALISE ACCEL TO HORIZON ===== */
    readAccel();

    // Calculate initial roll and pitch
    float aRollRadians = atan2f(ay, az);
    float aPitchRadians = atan2f(- ax, sqrtf(ay * ay + az * az));

    estRoll = aRollRadians * (180.0f / 3.14159265f);
    estPitch = aPitchRadians * (180.0f / 3.14159265f);
}

IMU::Attitude IMU::update(float dt) {

    // Read new data
    readGyro();
    readAccel();

    // Calculate new roll and pitch
    float aRollRadians = atan2f(ay, az);
    float aPitchRadians = atan2f(- ax, sqrtf(ay * ay + az * az));

    float aRoll = aRollRadians * (180.0f / 3.14159265f);
    float aPitch = aPitchRadians * (180.0f / 3.14159265f);

    // Complementary Filter
    IMU::Attitude resultData;

    estRoll = 0.98f*(estRoll + ((gx - rollOffset) * dt)) + 0.02f*aRoll;
    estPitch = 0.98f*(estPitch + ((gy - pitchOffset) * dt)) + 0.02f*aPitch;
    
    resultData.roll = estRoll;
    resultData.pitch = estPitch;
    resultData.yawRate = gz - yawOffset;

    return resultData;
}