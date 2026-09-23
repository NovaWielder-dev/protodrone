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
void setup() {
  
}

void loop() {

}