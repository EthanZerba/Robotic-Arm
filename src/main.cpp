#include <Arduino.h>
#include <AccelStepper.h>

// Stepper Motor Definitions
AccelStepper stepper1(AccelStepper::DRIVER, 5, 18); // Stepper motor 1 using DRIVER mode
AccelStepper stepper2(AccelStepper::DRIVER, 19, 21); // Stepper motor 2 using DRIVER mode
AccelStepper stepper3(AccelStepper::DRIVER, 22, 23); // Stepper motor 3 using DRIVER mode
AccelStepper stepper4(AccelStepper::FULL4WIRE, 12, 12, 14, 27); // 28BYJ-48 motor with ULN2003 driver

// Arm Segment Lengths (in mm)
const float L1 = 110.0; // Base to first joint
const float L2 = 255.0; // First to second joint
const float L3 = 240.0; // Second to third joint
const float L4 = 50.0;  // Third joint to end effector

// Steps per revolution for stepper motors
const int stepsPerRevolution = 200 * 16; // For NEMA 17 stepper motors (with microstepping)
const int stepsPerRevolution28BYJ48 = 4096; // For the 28BYJ-48 stepper motor

// Joint Angle Limits (in degrees)
const float THETA_1_MIN = 0.0, THETA_1_MAX = 180.0; // Limits for joint 1
const float THETA_2_MIN = -45.0, THETA_2_MAX = 225.0; // Limits for joint 2
const float THETA_3_MIN = -30.0, THETA_3_MAX = 190.0; // Limits for joint 3
const float THETA_4_MIN = 0.0, THETA_4_MAX = 360.0; // Limits for joint 4

// Maximum reach and height (in mm)
const float MAX_REACH = 520.0; // Maximum horizontal reach
const float MIN_HEIGHT = 0.0, MAX_HEIGHT = 630.0; // Minimum and maximum height

// Global variables to store current position of each stepper motor
long currentPositionStepper1 = 0;
long currentPositionStepper2 = 0;
long currentPositionStepper3 = 0;
long currentPositionStepper4 = 0;

// Calculated Angles
float Theta_1, Theta_2, Theta_3, Theta_4; // Variables to store the calculated joint angles
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
    stepper1.setMaxSpeed(500);
    stepper1.setAcceleration(250);
    
    stepper2.setMaxSpeed(7500);
    stepper2.setAcceleration(3500);
    
    stepper3.setMaxSpeed(500);
    stepper3.setAcceleration(250);
    
    stepper4.setMaxSpeed(500);
    stepper4.setAcceleration(250);
}
void moveSteppersToCalculatedPositions() {
    int steps1 = (Theta_1 * stepsPerRevolution) / 360.0;
    int steps2 = (Theta_2 * stepsPerRevolution * 26.85) / 360.0; // Gearbox adjustment
    int steps3 = (Theta_3 * stepsPerRevolution) / 360.0;
    int steps4 = (Theta_4 * stepsPerRevolution28BYJ48) / 360.0; // Adjusted for 28BYJ-48

    // Move to new positions
    stepper1.moveTo(currentPositionStepper1 + steps1);
    stepper2.moveTo(currentPositionStepper2 + steps2);
    stepper3.moveTo(currentPositionStepper3 + steps3);
    stepper4.moveTo(currentPositionStepper4 + steps4);

    // Update current positions
    currentPositionStepper1 += steps1;
    currentPositionStepper2 += steps2;
    currentPositionStepper3 += steps3;
    currentPositionStepper4 += steps4;

    // Run all steppers to their target positions simultaneously
    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || 
           stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
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
    Serial.println("Enter coordinates in the format X,Y,Z:");

}

void loop() {
    static String inputBuffer = ""; // Buffer to accumulate input characters
    if (Serial.available() > 0) {
        char receivedChar = Serial.read(); // Read the incoming character
        if (receivedChar == '\n') { // Check if the character is newline (Enter key)
            // Split the buffered string into X, Y, and Z coordinates
            int commaIndex1 = inputBuffer.indexOf(',');
            int commaIndex2 = inputBuffer.indexOf(',', commaIndex1 + 1);
            
            float X = inputBuffer.substring(0, commaIndex1).toFloat();
            float Y = inputBuffer.substring(commaIndex1 + 1, commaIndex2).toFloat();
            float Z = inputBuffer.substring(commaIndex2 + 1).toFloat();
            
            // Debug print the parsed coordinates
            Serial.print("Moving to X: ");
            Serial.print(X);
            Serial.print(" Y: ");
            Serial.print(Y);
            Serial.print(" Z: ");
            Serial.println(Z);
            
            // Calculate inverse kinematics and move
            calculateIK(X, Y, Z);
            moveSteppersToCalculatedPositions();
            
            // Clear the buffer after processing
            inputBuffer = "";
            
            // Ask for the next coordinates
            Serial.println("Enter coordinates in the format X,Y,Z:");
        } else {
            // If not newline, accumulate the character into the buffer
            inputBuffer += receivedChar;
        }
    }
}