#include <Arduino.h>
#include <AccelStepper.h>

    // Steps per revolution for stepper motors
const int stepsPerRevolution1 = 200 * 16; // For NEMA 17 stepper motors (with microstepping)
const int stepsPerRevolution2 = 200*16*26.84; // For NEMA 17 stepper motors (with microstepping) and a 26.85:1 gearbox
const int stepsPerRevolution3 = 200 * 16; // For NEMA 17 stepper motors (with microstepping)
const int stepsPerRevolution28BYJ48 = 4096; // For the 28BYJ-48 stepper motor

// For example, using a driver with step and direction pins connected to pins 2 and 3
AccelStepper stepper(AccelStepper::FULL4WIRE, 13, 12, 14, 27); // 28BYJ-48 motor with ULN2003 driver

void setup() {
  // Set the maximum speed and acceleration:
  stepper.setMaxSpeed(250); // steps per second
  stepper.setAcceleration(100); // steps per second squared

  // Calculate the number of steps for 90 degrees rotation
  int stepsFor90Degrees = (stepsPerRevolution28BYJ48 / 4);
  
  // Move the stepper motor 90 degrees
  stepper.move(stepsFor90Degrees);
}

void loop() {
  // Continuously run the stepper motor
  if (stepper.distanceToGo() == 0) {
    // If the motor has reached the target position, stop the motor
    stepper.stop();
  } else {
    // Otherwise, keep moving the motor towards the target position
    stepper.run();
  }
}