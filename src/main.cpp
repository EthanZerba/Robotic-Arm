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
float currentTheta1, currentTheta2, currentTheta3, currentTheta4; // Variables to store the current joint angles

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

    // Update the current angles to the new angles after movement
    currentTheta1 = Theta_1;
    currentTheta2 = Theta_2;
    currentTheta3 = Theta_3;

    // Run the steppers to reach the target positions
    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
    }

    // Update global position variables assuming the target was reached
    currentPositionStepper1 += steps1;
    currentPositionStepper2 += steps2;
    currentPositionStepper3 += steps3;

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

    // Base rotation
    Theta_1 = atan2(Y, X) * (180.0 / PI);

    // Effective horizontal and vertical distances
    float r = sqrt(X * X + Y * Y);
    float dx = r; 
    float dy = Z;

    // Handle special fully extended cases
    if (dx == MAX_REACH && dy == 0) { // Fully extended horizontally
        if (X > 0) {
            Theta_1 = 0; // Positive X
        } else {
            Theta_1 = 180; // Negative X
        }
        Theta_2 = 0;
        Theta_3 = 0;
    } else if (dy == MAX_HEIGHT && dx == 0) { // Fully extended vertically
        Theta_1 = 0; // Theta_1 does not matter, could set to initial
        Theta_2 = 90;
        Theta_3 = 0;
    } else {
        // General case using the cosine laws
        float D = sqrt(dx * dx + dy * dy);
        float angle2 = acos((L2 * L2 + D * D - L3 * L3) / (2 * L2 * D));
        float angle3 = acos((L2 * L2 + L3 * L3 - D * D) / (2 * L2 * L3));

        Theta_2 = angle2 * (180.0 / PI);
        Theta_3 = (PI - angle3) * (180.0 / PI); // Adjusting based on the physical arrangement
    }

    Serial.print("Calculated Theta_1: ");
    Serial.println(Theta_1);
    Serial.print("Calculated Theta_2: ");
    Serial.println(Theta_2);
    Serial.print("Calculated Theta_3: ");
    Serial.println(Theta_3);

    // Set target positions for global use
    targetX = X;
    targetY = Y;
    targetZ = Z;
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
    static bool isReadyForNewInput = true;  // Flag to indicate readiness for new input

    if (isReadyForNewInput) {
        Serial.print("Current Position: X=");
        Serial.print(currentX);
        Serial.print(", Y=");
        Serial.print(currentY);
        Serial.print(", Z=");
        Serial.println(currentZ);
        Serial.println("Enter new coordinates in the format X,Y,Z:");
        isReadyForNewInput = false;  // Reset flag until next input is processed
    }

    if (Serial.available() > 0) {
        char inChar = (char)Serial.read();  // Read the incoming character
        Serial.print(inChar);  // Echo the character back to the terminal

        if (inChar == '\r' || inChar == '\n') {  // Check for carriage return or newline (Enter key)
            Serial.println();  // Move to a new line
            float X, Y, Z;
            if (parseInput(inputBuffer, X, Y, Z)) {
                calculateIK(X, Y, Z);  // Calculate inverse kinematics for new target
                moveSteppersToCalculatedPositions();  // Move steppers to the new calculated positions
                isReadyForNewInput = true;  // Set flag to true as the robot is ready for new input
            } else {
                Serial.println("Invalid input. Please enter coordinates in the format X,Y,Z.");
                isReadyForNewInput = true;  // Allow retrying input
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