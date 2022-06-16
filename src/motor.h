/**
  motor.h - A file that contains all stepper motor controls
  Author: Jason Chen

  This file contains all stepper motor controls, which includes, initializing
  the stepper driver (TMCStepper), stepper motor control (FastAccelStepper), as
  well as recalling its current position and maximum position on reboot. It also
  sends current position via MQTT after it stops.

  It also gives the user the option to set the maximum and minimum stepper motor
  positions via MQTT. (1) User doesn't have to pre-calculate the max/min travel
  distance (2) User can still re-adjust max/min positions if the stepper motor
  slips.
**/
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <Preferences.h>
#include "motor_settings.h"

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDR);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Preferences memory;
void sendMqtt(String);


int maxPos;
int currPos;
int prevPos = 0;
bool isSetMax = false;
bool isSetMin = false;
bool isMotorRunning = false;
bool isMotorStalled = false;


void IRAM_ATTR stallguardInterrupt() {
  isMotorRunning = false;
  isMotorStalled = true;
  Serial.println("[I] Motor stalled");
}


void loadPositions() {
  memory.begin("local", false);
  maxPos = memory.getInt("maxPos", 50000);
  currPos = memory.getInt("currPos", 0);
  stepper.setCurrentPosition(currPos);
}


// TODO: add driver.shaft(bool) so user can reverse directions
void motorSetup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIAG_PIN, INPUT);

  // Stepper driver setup
  SERIAL_PORT.begin(115200);  // Initialize hardware serial for hardware UART driver
  driver.begin();             // Begin sending data
  driver.toff(4);             // Not used in StealthChop but required to enable the motor, 0=off
  driver.pdn_disable(true);   // PDN_UART input disabled; set this bit when using the UART interface
  driver.rms_current(600);    // Motor RMS current "rms_current will by default set ihold to 50% of irun but you can set your own ratio with additional second argument; rms_current(1000, 0.3)."
  driver.pwm_autoscale(true);    // Needed for StealthChop
  driver.en_spreadCycle(false);  // Disable SpreadCycle; SC is faster but louder
  driver.blank_time(24);         // Comparator blank time. Needed to safely cover the switching event and the duration of the ringing on the sense resistor.
  driver.microsteps(microsteps);

  #ifdef DIAG_PIN
  #ifdef RXD2
    driver.semin(0);    // CoolStep/SmartEnergy 4-bit uint that sets lower threshold, 0 disable
    driver.TCOOLTHRS((3089838.00 * pow(float(max_speed), -1.00161534)) * 1.5);  // Lower threshold velocity for switching on smart energy CoolStep and StallGuard to DIAG output
    driver.SGTHRS(10);  // [0..255] the higher the more sensitive to stall
    attachInterrupt(DIAG_PIN, stallguardInterrupt, RISING);
  #endif
  #endif

  // Stepper motor setup
  stepper.setEnablePin(EN_PIN);
  stepper.setMaxSpeed(max_speed);
  stepper.setAcceleration(acceleration);
  stepper.setPinsInverted(false, false, true);
  stepper.disableOutputs();

  // Load current position and maximum position from memory
  loadPositions();
}


// Returns current rounded position percentage. 0 is closed.
int motorCurrentPercent() {
  if (currPos == 0)
    return 0;
  else
    return (int) round((float) currPos / (float) maxPos * 100);
}


void motorStop() {
  isMotorRunning = false;

  // Re-calcuate max/min positions
  if (isSetMax) {
    isSetMax = false;
    maxPos = stepper.currentPosition();
    memory.putInt("maxPos", maxPos);
    stepper.setMaxSpeed(max_speed);  // Set stepper motor speed back to normal
  } else if (isSetMin) {
    isSetMin = false;
    int distanceTraveled = 2147483646 - stepper.currentPosition();
    maxPos = maxPos + distanceTraveled - prevPos;
    stepper.setCurrentPosition(0);
    stepper.setMaxSpeed(max_speed);  // Set stepper motor speed back to normal
  }

  // Stop stepper motor and disable driver
  stepper.moveTo(stepper.currentPosition());
  stepper.disableOutputs();

  // Updated current position
  currPos = stepper.currentPosition();
  memory.putInt("currPos", currPos);

  // Send current position in percentage to MQTT server
  sendMqtt((String) motorCurrentPercent());
}


void motorRun() {
  if (isMotorRunning) {
    if (stepper.distanceToGo() != 0)
      stepper.run();
    else
      motorStop();
  }

  if (isMotorStalled) {
    isMotorStalled = false;
    motorStop();
  }
}


int percentToSteps(int percent) {
  float steps = (float) percent * (float) maxPos / 100.0;
  return (int) round(steps);
}


// Stepper must move first before isMotorRunning==true; else motorRun will excute first before stepper stops
void motorMoveTo(int newPos) {
  stepper.moveTo(newPos);
  if (stepper.distanceToGo() != 0 && newPos <= maxPos) {
    isMotorRunning = true;
    stepper.enableOutputs();
  }
}


void motorMove(int percent) {
  motorMoveTo(percentToSteps(percent));
}


void motorMin() {
  motorMoveTo(0);
}


void motorMax() {
  motorMoveTo(maxPos);
}


void motorSetMax() {
  isSetMax = true;
  stepper.setMaxSpeed(max_speed / 4);
  maxPos = 2147483646;
  motorMoveTo(2147483646);
}


void motorSetMin() {
  isSetMin = true;
  stepper.setMaxSpeed(max_speed / 4);
  prevPos = stepper.currentPosition();
  stepper.setCurrentPosition(2147483646);
  motorMoveTo(0);
}