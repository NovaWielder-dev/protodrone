# ProtoDrone: ESP32 Custom Brushless Quadcopter

![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)

A custom, ground-up quadcopter flight controller and airframe project built on an ESP32. The goal is to develop clean, modular embedded flight software and control loops from scratch in modern C++ (C++17), prioritizing engineering principles and control theory over off-the-shelf flight stacks.

The target flight envelope is basic autonomous stabilization: takeoff, stable hover, manual attitude/thrust tracking, and landing.

---

## Hardware Specifications

* **Microcontroller:** ESP32 (Dual-core Xtensa LX6, 240 MHz, hardware single-precision 32-bit FPU).
* **IMU:** MPU-6050 (6-DOF Accelerometer + Gyroscope).
  * **Bus:** I2C configured for 400 kHz Fast Mode.
  * **Pins:** `SDA = GPIO 21`, `SCL = GPIO 22`.
* **Motors:** 4× Mitoot 1806 2280kV brushless DC motors (~300–450 g thrust/motor on 4–5" propellers; ~1.2–1.8 kg total static thrust).
* **ESCs:** 4× standalone brushless ESCs.
  * **Protocol:** Standard Servo PWM (50 Hz frame rate, 16-bit LEDC timer resolution, 1000 µs idle to 2000 µs maximum pulse duration).
* **Frame:** Custom 3D-printed in PLA. Features a central rectangular avionics/battery bay with raised corner tower mounts accepting arms via top-slot T-joints.
* **Secondary / Deferred Peripherals:**
  * GPS module (on hand, deferred until inner stabilization loop is verified).
  * Ground Link: ESP-NOW protocol over 2.4 GHz RF or direct UDP bridge from a Python/Pygame gamepad station.

### Motor Geometry ("X" Configuration)

```text
          [FRONT]
     M1 (CW)     M2 (CCW)
         \         /
          \       /
           \     /
          /       \
         /         \
        /           \
     M4 (CCW)    M3 (CW)
          [REAR]