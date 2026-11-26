#include "PID.h"

PID::PID(float Kp, float Ki, float Kd, float maxI, float maxOut)
    : Kp(Kp), Ki(Ki), Kd(Kd), maxI(maxI), maxOut(maxOut)
{
    I_sum = 0.0f;
    hasPrev = false;
    prevMeasured = 0.0f;
}

float PID::update(float target, float measured, float dt) {
    
    // heading += dt * 5d/s
    // Integral_gain += Ki * dt * heading
    // error = (Kp * heading) + (integral_gain) + (Kd * avel)
    
    // Current error
    float e_curr = target - measured;

    // Proportional (P)
    float P = Kp * e_curr;

    // Derivative (D)
    float D = 0.0f; // Default in case of liftoff
    if (hasPrev && dt > 0.0f) {
        D = -Kd * ((measured - prevMeasured) / dt);
    }
    prevMeasured = measured;
    hasPrev = true;

    // Integral (I)
    I_sum += e_curr * dt;
    if (I_sum > maxI) { // Cap I_sum; over bound
        I_sum = maxI; 
    }
    else if (I_sum < -maxI) {
        I_sum = -maxI; // Round I_sum; under bound
    }
    float I = Ki * I_sum;

    // Return clamped sum
    if (P + I + D > maxOut) {return maxOut;}
    else if (P + I + D < -maxOut) {return -maxOut;}
    return P + I + D;
}

void PID::reset() {
    I_sum = 0.0f;
    hasPrev = false;
    prevMeasured = 0.0f;
}