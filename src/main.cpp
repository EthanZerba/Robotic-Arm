#include <Arduino.h>
#include <AccelStepper.h>

// Stepper Motor Definitions
AccelStepper stepper1(AccelStepper::DRIVER, 5, 18); // Stepper motor 1 using DRIVER mode
AccelStepper stepper2(AccelStepper::DRIVER, 19, 21); // Stepper motor 2 using DRIVER mode
AccelStepper stepper3(AccelStepper::DRIVER, 22, 23); // Stepper motor 3 using DRIVER mode
AccelStepper stepper4(AccelStepper::FULL4WIRE, 13, 12, 14, 27); // 28BYJ-48 motor with ULN2003 driver

float currentX = 495.0;
float currentY = 0.0;
float currentZ = 0.0;

float targetX, targetY, targetZ;

// Arm Segment Lengths (in mm)
const float L1 = 110.0; // Base to first joint
const float L2 = 255.0; // First to second joint
const float L3 = 240.0; // Second to third joint
const float L4 = 0.0;  // Third joint to end effector

// Steps per revolution for stepper motors
const int stepsPerRevolution = 200 * 16; // For NEMA 17 stepper motors (with microstepping)
const int stepsPerRevolution28BYJ48 = 4096; // For the 28BYJ-48 stepper motor

// Joint Angle Limits (in degrees)
const float THETA_1_MIN = 0.0, THETA_1_MAX = 180.0; // Limits for joint 1
const float THETA_2_MIN = -45.0, THETA_2_MAX = 225.0; // Limits for joint 2
const float THETA_3_MIN = -30.0, THETA_3_MAX = 190.0; // Limits for joint 3
const float THETA_4_MIN = 0.0, THETA_4_MAX = 360.0; // Limits for joint 4

// Maximum reach and height (in mm)
const float MAX_REACH = 495.0; // Maximum horizontal reach
const float MIN_HEIGHT = 0.0, MAX_HEIGHT = 495.0; // Minimum and maximum height

// Global variables to store current position of each stepper motor
long currentPositionStepper1, currentPositionStepper2, currentPositionStepper3, currentPositionStepper4;

// Calculated Angles
float Theta_1, Theta_2, Theta_3, Theta_4; // Variables to store the calculated joint angles

void moveSteppersToCalculatedPositions() {
    int steps1 = (Theta_1 * stepsPerRevolution) / 360.0;
    int steps2 = (Theta_2 * stepsPerRevolution) / 360.0;
    int steps3 = (Theta_3 * stepsPerRevolution) / 360.0;
    int steps4 = (Theta_4 * stepsPerRevolution28BYJ48) / 360.0;

    stepper1.moveTo(currentPositionStepper1 + steps1);
    stepper2.moveTo(currentPositionStepper2 + steps2);
    stepper3.moveTo(currentPositionStepper3 + steps3);
    stepper4.moveTo(currentPositionStepper4 + steps4);

    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || 
           stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0) {
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

void calculateIK(float X, float Y, float Z) {
    Serial.print("Received Target: X=");
    Serial.print(X);
    Serial.print(", Y=");
    Serial.print(Y);
    Serial.print(", Z=");
    Serial.println(Z);

    // Directly calculate angles based on the target position
    float angle1 = atan2(Y, X) * (180.0 / PI);
    Theta_1 = angle1;
    Serial.print("Calculated Theta_1: ");
    Serial.println(Theta_1);

    float dx = sqrt(X * X + Y * Y);
    float dy = Z;  // Assuming Z includes the height of the first joint
    Serial.print("Intermediate dx: ");
    Serial.println(dx);
    Serial.print("Intermediate dy: ");
    Serial.println(dy);

    float D = sqrt(dx * dx + dy * dy);
    Serial.print("Intermediate D: ");
    Serial.println(D);

    // Calculate Theta_2 based on the position
    if (dx == 0 && dy > 0) {
        Theta_2 = 90;  // Directly set to 90 degrees if directly above the base
    } else {
        float cosAngle2 = (L2 * L2 + D * D - L3 * L3) / (2 * L2 * D);
        cosAngle2 = constrain(cosAngle2, -1.0, 1.0);
        Theta_2 = acos(cosAngle2) * (180.0 / PI);
    }
    Serial.print("Calculated Theta_2: ");
    Serial.println(Theta_2);

    // Calculate Theta_3 based on the position
    if (D == L2) {
        Theta_3 = 0;  // If the third segment should align straight with the second
    } else {
        float cosAngle3 = (L2 * L2 + L3 * L3 - D * D) / (2 * L2 * L3);
        cosAngle3 = constrain(cosAngle3, -1.0, 1.0);
        Theta_3 = 180 - acos(cosAngle3) * (180.0 / PI);
    }
    Serial.print("Calculated Theta_3: ");
    Serial.println(Theta_3);

    // Set target positions for global use
    targetX = X;
    targetY = Y;
    targetZ = Z;

    moveSteppersToCalculatedPositions();
}


void initializeSteppers() {
    // Setup for stepper motors with speed and acceleration
    stepper1.setMaxSpeed(250);
    stepper1.setAcceleration(50);
    stepper2.setMaxSpeed(1000);
    stepper2.setAcceleration(250);
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

void setup() {
    Serial.begin(9600);
    initializeSteppers();

    // Set the current position of each stepper to correspond to the initial position
    stepper1.setCurrentPosition(0); // This corresponds to the robot facing along the positive X-axis
    stepper2.setCurrentPosition(0); // Initial angles for other steppers
    stepper3.setCurrentPosition(0);
    stepper4.setCurrentPosition(0);

    Serial.println("Calibration assumed at 495,0,0. Ready for commands.");
    Serial.println("Enter coordinates in the format X,Y,Z:");
}

bool parseInput(const String& input, float& X, float& Y, float& Z) {
    int commaIndex1 = input.indexOf(',');
    int commaIndex2 = input.indexOf(',', commaIndex1 + 1);
    if (commaIndex1 == -1 || commaIndex2 == -1) return false;

    X = input.substring(0, commaIndex1).toFloat();
    Y = input.substring(commaIndex1 + 1, commaIndex2).toFloat();
    Z = input.substring(commaIndex2 + 1).toFloat();
    return true;
}

void loop() {
    static String inputBuffer = "";  // Buffer to store input characters
    if (Serial.available() > 0) {
        char inChar = (char)Serial.read();  // Read the incoming character
        Serial.print(inChar);  // Echo the character back to the terminal

        if (inChar == '\r') {  // Check for carriage return (Enter key)
            Serial.println();  // Move to a new line
            float X, Y, Z;
            if (parseInput(inputBuffer, X, Y, Z)) {
                calculateIK(X, Y, Z);
                moveSteppersToCalculatedPositions();
            }
            inputBuffer = "";  // Clear the buffer after processing
        } else if (inChar == '\b') {  // Handle backspace
            if (inputBuffer.length() > 0) {
                inputBuffer.remove(inputBuffer.length() - 1);  // Remove last character from buffer
                Serial.print("\b \b");  // Erase the last character on the terminal
            }
        } else {
            inputBuffer += inChar;  // Add character to buffer
        }
    }
}

