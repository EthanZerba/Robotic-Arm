#include "StepperControl.h"
#include "InverseKinematics.h"
#include "AccelStepper.h"
#include "GlobalVars.h"

extern AccelStepper stepper1, stepper2, stepper3; // Assuming these are defined elsewhere
// Global variables to store current position of each stepper motor
long currentPositionStepper1, currentPositionStepper2, currentPositionStepper3, currentPositionStepper4;
float currentTheta1, currentTheta2, currentTheta3, currentTheta4;




// Steps per revolution for stepper motors
const int stepsPerRevolution = 200 * 16; // For NEMA 17 stepper motors (with microstepping)
const int stepsPerRevolution28BYJ48 = 4096; // For the 28BYJ-48 stepper motor

// Stepper Motor Definitions
AccelStepper stepper1(AccelStepper::DRIVER, 5, 18); // Stepper motor 1 using DRIVER mode
AccelStepper stepper2(AccelStepper::DRIVER, 19, 21); // Stepper motor 2 using DRIVER mode
AccelStepper stepper3(AccelStepper::DRIVER, 22, 23); // Stepper motor 3 using DRIVER mode
AccelStepper stepper4(AccelStepper::FULL4WIRE, 13, 12, 14, 27); // 28BYJ-48 motor with ULN2003 driver

void disableSteppers() {
    stepper1.disableOutputs();
    stepper2.disableOutputs();
    stepper3.disableOutputs();
    stepper4.disableOutputs();
}

void enableSteppers() {
    stepper1.enableOutputs();
    stepper2.enableOutputs();
    stepper3.enableOutputs();
    stepper4.enableOutputs();
}

void moveSteppersToCalculatedPositions() {
    Serial.println("Moving to the calculated positions...");
    Serial.print(Theta_1);
    Serial.print(", ");
    Serial.println(currentTheta1);
   
    Serial.print(Theta_2);
    Serial.print(", ");
    Serial.println(currentTheta2);

    Serial.print(Theta_3);
    Serial.print(", ");
    Serial.println(currentTheta3);

    Serial.print(Theta_4);
    Serial.print(", ");
    Serial.println(currentTheta4);

    int steps1 = ((Theta_1 - currentTheta1) * stepsPerRevolution) / 360.0;
    int steps2 = ((Theta_2 - currentTheta2) * stepsPerRevolution * 26.85) / 360.0;
    int steps3 = ((Theta_3 - currentTheta3) * stepsPerRevolution) / 360.0;
    int steps4 = ((Theta_4 - currentTheta4) * stepsPerRevolution28BYJ48) / 360.0;

    // Move the steppers by the calculated step differences
    stepper1.move(steps1);
    stepper2.move(steps2);
    stepper3.move(steps3);
    stepper4.move(steps4);

    // Update the current angles to the new angles after movement
    currentTheta1 = Theta_1;
    currentTheta2 = Theta_2;
    currentTheta3 = Theta_3;
    currentTheta4 = Theta_4;

    // Run the steppers to reach the target positions
    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
        stepper4.run();

    }

    // Update global position variables assuming the target was reached
    currentPositionStepper1 += steps1;
    currentPositionStepper2 += steps2;
    currentPositionStepper3 += steps3;
    currentPositionStepper4 += steps4;

    currentX = targetX;
    currentY = targetY;
    currentZ = targetZ;
}

void initializeSteppers() {
    // Setup for stepper motors with speed and acceleration
    stepper1.setMaxSpeed(250);
    stepper1.setAcceleration(50);
    stepper2.setMaxSpeed(6712);
    stepper2.setAcceleration(1347);
    stepper3.setMaxSpeed(250);
    stepper3.setAcceleration(50);
    stepper4.setMaxSpeed(500);
    stepper4.setAcceleration(250);

    // Set current positions assuming initial calibration at 0 degrees
    stepper1.setCurrentPosition(0);
    stepper2.setCurrentPosition(0);
    stepper3.setCurrentPosition(0);
    stepper4.setCurrentPosition(0);

    

    // Initial angles for steppers based on the physical setup
    Theta_1 = 0;
    Theta_2 = 0;
    Theta_3 = 0;
    Theta_4 = 0;
}
