#include "PID.h"

PID::PID(float Kp, float Ki, float Kd, float maxI, float maxOut)
    : Kp(Kp), Ki(Ki), Kd(Kd), maxI(maxI), maxOut(maxOut)
{
    I_sum = 0.0f;
    e_prev = 0.0f;
}

float PID::update(float setPoint, float measured, float dt) {
    // Current error
    float e_curr = setPoint - measured;

    // Proportional (P)
    float P = Kp * e_curr;

    // Derivative (D)
    float D = 0.0f; // Assuming no change to error
    if (dt != 0.0f) {
        D = Kd * ((e_curr - e_prev) / dt);
    }

    // Integral (I)
    I_sum += e_curr * dt;
    if (I_sum > maxI) { // Cap I_sum; over bound
        I_sum = maxI; 
    }
    else if (I_sum < -maxI) {
        I_sum = -maxI; // Round I_sum; under bound
    }
    float I = Ki * I_sum;

    // Update previous error
    e_prev = e_curr;

    // Return clamped sum
    if (P + I + D > maxOut) {return maxOut;}
    else if (P + I + D < -maxOut) {return -maxOut;}
    return P + I + D;
}

void PID::reset() {
    I_sum = 0.0f;
    e_prev = 0.0f;
}