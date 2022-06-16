/**
  motor_settings.h - Specific hardware settings for the stepper driver and motor
  Author: Jason Chen

  A file that includes the hardware settings for stepper driver (BTT TMC2209 V1.2)
  and stepper motor (NEMA 11).

  To use this file:
  - Replace the connections from ESP32 to TMC2209 (mainly EN_PIN, STEP_PIN, DIR_PIN)
  - If you don't plan on using stallguard, comment out DIAG_PIN and RXD2
  - Check and modify stepper motor specifications. I use a NEMA11 with a 5.18 planetary
    gearbox because that gives me enough torque to move the shades while being small
    enough to fit inside the top compartment.
**/

// Define ESP32 connections to stepper motor driver (BigTreeTech TMC2209 V1.2 UART)
#define EN_PIN           23 // Enable pin
#define STEP_PIN         32 // Step pin
#define DIR_PIN          21 // Direction pin
#define DIAG_PIN         35 // StallGuard (diag) pin
#define RXD2             17 // UART receive, must be connected to a UART port on the ESP32
// #define TXD2             22 // UART transmission; one with the 1k ohm resistor
#define SERIAL_PORT Serial2 // TMC2209 HardwareSerial port
#define R_SENSE       0.11f // Sense resistor, TMC2209 uses 0.11
#define DRIVER_ADDR    0b00 // 0b00 is slave, since there're no other drivers

// Stepper motor specifications (NEMA 11 with 5.18:1 planetary gearbox)
const float gearbox_ratio = 5.18;         // Use 1 if stepper motor doesn't have a gearbox
const uint16_t microsteps = 8;            // 8 microsteps per full step
const int steps_per_rev = 200 * gearbox_ratio * microsteps;  // NEMA motors have 200 full steps per rev
const int max_speed = steps_per_rev * 1;  // Max speed in Hz
const int acceleration = max_speed * 2;   // Use lower val so it doesn't trip StallGuard when accelerating