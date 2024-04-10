#include <Arduino.h>
#include <AccelStepper.h>

// Stepper Motor Definitions
AccelStepper stepper1(AccelStepper::DRIVER, 18, 5);
AccelStepper stepper2(AccelStepper::DRIVER, 21, 19);
AccelStepper stepper3(AccelStepper::DRIVER, 23, 22);
AccelStepper stepper4(AccelStepper::FULL4WIRE, 8, 10, 9, 11);
// Arm Segment Lengths (in mm)
const float L1 = 110.0;
const float L2 = 260.0;
const float L3 = 240.0;
const float L4 = 10.0;

// Steps Per Revolution (with microstepping)
const int stepsPerRevolution = 200 * 16;
// Steps per revolution for 28BYJ-48
const int stepsPerRevolution28BYJ48 = 4096;

// Joint Angle Limits (in degrees)
const float THETA_1_MIN = -360, THETA_1_MAX = 360.0; // Example limits for joint 1
const float THETA_2_MIN = -180, THETA_2_MAX = 0; // Example limits for joint 2
const float THETA_3_MIN = -200.0, THETA_3_MAX = 200.0;
const float THETA_4_MIN = -360, THETA_4_MAX = 360.0;
// Reach and Height Limits (in mm)
const float MAX_REACH = 510.0;
const float MIN_HEIGHT = 0.0, MAX_HEIGHT = 620.0;

// Calculated Angles
float Theta_1, Theta_2, Theta_3, Theta_4;

void calculateIK(float X, float Y, float Z) {
    float distance = sqrt(X*X + Y*Y);
    if (distance > MAX_REACH || Z < MIN_HEIGHT || Z > MAX_HEIGHT) {
        Serial.println("Error: Target position out of reach.");
        return;
    }

    Theta_1 = atan2(Y, X) * (180.0 / PI);
    float D = distance - L1;
    float H = Z - L1;
    float L_target = sqrt(D*D + H*H);
    float acosArg1 = (L2*L2 + L_target*L_target - L3*L3) / (2 * L2 * L_target);
    float acosArg2 = (L2*L2 + L3*L3 - L_target*L_target) / (2 * L2 * L3);
    acosArg1 = constrain(acosArg1, -1.0, 1.0);
    acosArg2 = constrain(acosArg2, -1.0, 1.0);

    float angle_L2_Ltarget = acos(acosArg1);
    float angle_L3_Ltarget = acos(acosArg2);
    float phi = atan2(H, D);

    Theta_2 = (phi + angle_L2_Ltarget) * (180.0 / PI);
    Theta_3 = (angle_L3_Ltarget - phi) * (180.0 / PI);
    Theta_4 = -(Theta_2 + Theta_3);

    Theta_1 = constrain(Theta_1, THETA_1_MIN, THETA_1_MAX);
    Theta_2 = constrain(Theta_2, THETA_2_MIN, THETA_2_MAX);
    Theta_3 = constrain(Theta_3, THETA_3_MIN, THETA_3_MAX);
    Theta_4 = constrain(Theta_4, THETA_4_MIN, THETA_4_MAX);
}

void initializeSteppers() {
    stepper1.setMaxSpeed(1000);
    stepper1.setAcceleration(500);
    
    stepper2.setMaxSpeed(1000);
    stepper2.setAcceleration(500);
    
    stepper3.setMaxSpeed(1000);
    stepper3.setAcceleration(500);
    
    stepper4.setMaxSpeed(1000);
    stepper4.setAcceleration(500);
}
void moveSteppersToCalculatedPositions() {
    int steps1 = (Theta_1 * stepsPerRevolution) / 360.0;
    int steps2 = (Theta_2 * stepsPerRevolution * 27) / 360.0; // Gearbox adjustment
    int steps3 = (Theta_3 * stepsPerRevolution) / 360.0;
    int steps4 = (Theta_4 * stepsPerRevolution28BYJ48) / 360.0; // Adjusted for 28BYJ-48

    stepper1.moveTo(stepper1.currentPosition() + steps1);
    while (stepper1.distanceToGo() != 0) {
        stepper1.run();
    }
    
    stepper2.moveTo(stepper2.currentPosition() + steps2);
    while (stepper2.distanceToGo() != 0) {
        stepper2.run();
    }
    
    stepper3.moveTo(stepper3.currentPosition() + steps3);
    while (stepper3.distanceToGo() != 0) {
        stepper3.run();
    }
    
    stepper4.moveTo(stepper4.currentPosition() + steps4);
    while (stepper4.distanceToGo() != 0) {
        stepper4.run();
    }
}

void setup() {
    Serial.begin(9600);
    initializeSteppers();
    Serial.println("Manually extend the arm fully in the positive X direction. Press any key to confirm...");
    while (!Serial.available()) {}
    Serial.read();
    Serial.println("Calibration complete. The arm is now at the 0 reference position.");
}

void loop() {
    // Placeholder target position for demonstration
    calculateIK(100, 100, 0);
    moveSteppersToCalculatedPositions();
    // Add any necessary delay or additional logic for continuous operation
}