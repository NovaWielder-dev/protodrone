#include "Config.h"
#include "Motors.h"
#include <ESP32Servo.h>
#include <Arduino.h>

// Declare anonymous constant aliases (avoids verbosity) and Servo objects
namespace {

    constexpr int PIN_FL = Config::Motors::PIN_FRONT_LEFT;
    constexpr int PIN_FR = Config::Motors::PIN_FRONT_RIGHT;
    constexpr int PIN_RR = Config::Motors::PIN_REAR_RIGHT;
    constexpr int PIN_RL = Config::Motors::PIN_REAR_LEFT;
    
    constexpr int MIN_US = static_cast<int>(Config::Motors::MIN_PULSE_US);
    constexpr int MAX_US = static_cast<int>(Config::Motors::MAX_PULSE_US);

    Servo escFL, escFR, escRR, escRL;

}

// INITIALISE HARDWARE TIMERS + ATTACH PINS
void Motors::init() {
    
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    escFL.setPeriodHertz(50);
    escFR.setPeriodHertz(50);
    escRR.setPeriodHertz(50);
    escRL.setPeriodHertz(50);

    escFL.attach(PIN_FL, MIN_US, MAX_US);
    escFR.attach(PIN_FR, MIN_US, MAX_US);
    escRR.attach(PIN_RR, MIN_US, MAX_US);
    escRL.attach(PIN_RL, MIN_US, MAX_US);

    stop();
}

// SAFETY ARMING
void Motors::arm() {

    Serial.println("[ARMING] Holding esc's at minimum thrust for 3 sec...");
    stop();
    delay(3000);
}

// SET ALL ESC'S TO MINIMUM PULSE (ZERO THRUST)
void Motors::stop() {
    
    escFL.writeMicroseconds(MIN_US);
    escFR.writeMicroseconds(MIN_US);
    escRR.writeMicroseconds(MIN_US);
    escRL.writeMicroseconds(MIN_US);
}

// SEND MOTOR COMMANDS
void Motors::write(const Mixer::MotorCommands& cmds) {

    escFL.writeMicroseconds(static_cast<int>(cmds.fl));
    escFR.writeMicroseconds(static_cast<int>(cmds.fr));
    escRR.writeMicroseconds(static_cast<int>(cmds.rr));
    escRL.writeMicroseconds(static_cast<int>(cmds.rl));
}