#include "IMU.h"
#include "Motors.h"
#include "PID.h"
#include "Mixer.h"
#include "Receiver.h"

/* ========== GLOBAL VARIABLES ========== */

// PID(Kp, Ki, Kd, maxI, maxOut);
PID pitchPID(1.2f, 0.04f, 0.18f, 50.0f, 400.0f);
PID rollPID(1.2f, 0.04f, 0.18f, 50.0f, 400.0f);
PID yawPID(2.0f, 0.02f, 0.0f, 50.0f, 400.0f);

/* ========== OPERATION ========== */
#include <Wire.h>
#include <ESP32Servo.h>

// Define ESC objects
Servo esc1; // right front
Servo esc2; // left front
Servo esc3; // right back
Servo esc4; // left back

// Define ESC pins
const int esc1Pin = 25;
const int esc2Pin = 26;
const int esc3Pin = 32;
const int esc4Pin = 33;

const int gyro_sda = 22;
const int gyro_scl = 21;
float GyroOffset = 0;

float ax = 0;
float ay = 0;
float az = 0;

float gx = 0;
float gy = 0;
float gz = 0;

float aRoll;
float aPitch;

volatile float Roll = 0;
volatile float Pitch = 0;
volatile float Yaw = 0;

volatile float m1 = 0;
volatile float m2 = 0;
volatile float m3 = 0;
volatile float m4 = 0;

float rollRadians = 0;
float pitchRadians = 0;

float aRollOffset = 0;
float aPitchOffset = 0;

float gRollOffset = 0;
float gPitchOffset = 0;
float gYawOffset = 0;



void readGyro() {

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);

  if (Wire.requestFrom(0x68, 6) == 6) {
    int16_t rawX = (Wire.read() << 8 | Wire.read());
    int16_t rawY = (Wire.read() << 8 | Wire.read());
    int16_t rawZ = (Wire.read() << 8 | Wire.read());

    gx = rawX / 131.0;
    gy = rawY / 131.0;
    gz = rawZ / 131.0;
    
  }
}


void readAccelerometer() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  if (Wire.requestFrom(0x68, 6) == 6) {
    int16_t rawX = (Wire.read() << 8 | Wire.read());
    int16_t rawY = (Wire.read() << 8 | Wire.read());
    int16_t rawZ = (Wire.read() << 8 | Wire.read());

    ax = rawX / 16384.0;
    ay = rawY / 16384.0;
    az = rawZ / 16384.0;

    rollRadians = atan2(ay, az);
    pitchRadians = atan2(-ax, sqrt(ay * ay + az * az));
    aRoll = rollRadians * (180.0 / PI);
    aPitch = pitchRadians * (180.0 / PI);
  }
}

void calibrateAccel() {
  float totalPitch = 0;
  float totalRoll = 0;

  for (int i = 0; i < 100; i++) {
    readAccelerometer();
    totalRoll += aRoll;
    totalPitch += aPitch;
  }
  
  aRollOffset = totalRoll / 100.0;
  aPitchOffset = totalPitch / 100.0;

}

void calibrateGyro() {
  float totalX = 0;
  float totalY = 0;
  float totalZ = 0;

  for (int i = 0; i < 100; i++) {
    readGyro();
    totalX += gx;
    totalY += gy;
    totalZ += gz;
  }

  gRollOffset = totalX / 100.0;
  gPitchOffset = totalY / 100.0;
  gYawOffset = totalZ / 100.0;
}

void SerialOutput(void *pvParameters) {
  for (;;) {
    Serial.print("Roll: ");
    Serial.println(Roll);
    Serial.print("Pitch: ");
    Serial.println(Pitch);
    Serial.print("Yaw: ");
    Serial.println(Yaw);
    Serial.print("M1: ");
    Serial.println(m1);
    Serial.print("M2: ");
    Serial.println(m2);
    Serial.print("M3: ");
    Serial.println(m3);
    Serial.print("M4: ");
    Serial.println(m4);

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Waiting 3 seconds before startup...");
  delay(3000);
  Serial.println("Starting...");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // 4. Attach ESCs at 50Hz standard refresh rate
  esc1.setPeriodHertz(50);
  esc2.setPeriodHertz(50);
  esc3.setPeriodHertz(50);
  esc4.setPeriodHertz(50);
  
  esc1.attach(esc1Pin, 1000, 2000);
  esc2.attach(esc2Pin, 1000, 2000);
  esc3.attach(esc3Pin, 1000, 2000);
  esc4.attach(esc4Pin, 1000, 2000);



  Wire.begin(gyro_sda, gyro_scl);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    Serial.println("ERROR: MPU6050 not found! Check SDA/SCL wiring.");
  } else {
    Serial.println("Calibrating...");
    calibrateAccel();
    calibrateGyro();

  }

    xTaskCreatePinnedToCore(
    SerialOutput,
    "Serial output task",
    4096,
    NULL,
    1,
    NULL,
    0
  );

}

void PID() {
  Serial.println("Starting PID loop in 5 seconds...");
  delay(5000);
  Serial.println("PID loop starting, calculating current heading...");
  float totalPitch = 0;
  float totalRoll = 0;

  

  for (int i = 0; i < 100; i++) {
    readAccelerometer();
    totalRoll += aRoll;
    totalPitch += aPitch;
  }
  Roll = ((totalRoll / 100.0) - aRollOffset + 180);
  Pitch = ((totalPitch / 100.0) - aPitchOffset);
  Yaw = 0;

  float Kp = 0.1;
  float Ki = 1;
  float Kd = 2;

  float baseThrottle = 1500;

  Serial.println("Current heading calculated: ");
  Serial.print("Roll: ");
  Serial.println(Roll);
  Serial.print("Pitch: ");
  Serial.println(Pitch);
  Serial.print("Yaw: ");
  Serial.println(Yaw);

  unsigned long dtTimer = micros();

  while (true) {

    readGyro();
    readAccelerometer();
    float dt = (micros() - dtTimer) / 1000000.0;
    dtTimer = micros();

    float deltaRoll = dt * (gx - gRollOffset);
    float deltaPitch = dt * (gy - gPitchOffset);
    float deltaYaw = dt * (gz - gYawOffset);

    Roll += deltaRoll;
    Pitch += deltaPitch;
    Yaw += deltaYaw;

    float targetPitch = 0;
    float targetRoll = 0;

    float pitchError = targetPitch - Pitch;
    float rollError = targetRoll - Roll;

    float pPitch = pitchError * Kp;
    float pRoll = rollError * Kp;

    m1 = baseThrottle - pPitch - pRoll; // Front Right
    m2 = baseThrottle - pPitch + pRoll; // Front Left
    m3 = baseThrottle + pPitch - pRoll; // Back Right
    m4 = baseThrottle + pPitch + pRoll; // Back Left

  }
}

void loop() {
  PID();
}