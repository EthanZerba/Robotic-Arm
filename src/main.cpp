#include <Arduino.h>
#include <Stepper.h>

// Arm segment lengths (in mm)
float L1 = 25.0;  // Length from floor to the first axis of rotation
float L2 = 125.0; // Length from the first axis to the second axis
float L3 = 125.0; // Length from the second axis to the third axis
float L4 = 10.0;  // Length from the third axis to the end effector

// Range of motion for each joint (in degrees)
float THETA_1_MIN = 0.0, THETA_1_MAX = 360.0;
float THETA_2_MIN = 0.0, THETA_2_MAX = 360.0;
float THETA_3_MIN = 0.0, THETA_3_MAX = 360.0;
float THETA_4_MIN = 0.0, THETA_4_MAX = 360.0;

// Stepper motor setup for 28BYJ-48
const int stepsPerRevolution = 4096; // Adjust based on your stepper motor
Stepper stepper1(stepsPerRevolution, 8, 9, 10, 11); // Update pins as needed
Stepper stepper2(stepsPerRevolution, 4, 5, 6, 7);   // Update pins as needed
Stepper stepper3(stepsPerRevolution, 0, 1, 2, 3);   // Update pins as needed
Stepper stepper4(stepsPerRevolution, 12, 13, A0, A1); // Update pins as needed

// Variables to store the calculated angles
float Theta_1, Theta_2, Theta_3, Theta_4;

// Maximum reach of the arm (in mm)
const float MAX_REACH = 260.0; // L2 + L3 + some margin
// Minimum and maximum height the arm can reach (in mm)
const float MIN_HEIGHT = 0.0;
const float MAX_HEIGHT = 260.0; // Adjust based on your arm's capability

void calculateIK(float X, float Y, float Z) {
    // Validate inputs
    float distance = sqrt(X*X + Y*Y);
    if (distance > MAX_REACH || Z < MIN_HEIGHT || Z > MAX_HEIGHT) {
        Serial.println("Error: Target position out of reach.");
        return; // Exit the function if target is out of reach
    }

    // Calculate Theta_1 for base rotation
    Theta_1 = atan2(Y, X) * (180.0 / PI);

    // Project target onto the XZ plane
    float D = sqrt(X*X + Y*Y) - L1;
    float H = Z - L1; // Adjusted to account for the height of L1

    // Calculate the length of the line from the shoulder to the target (L_target)
    float L_target = sqrt(D*D + H*H);

    // Ensure arguments for acos are within the valid range [-1, 1]
    float acosArg1 = (L2*L2 + L_target*L_target - L3*L3) / (2 * L2 * L_target);
    float acosArg2 = (L2*L2 + L3*L3 - L_target*L_target) / (2 * L2 * L3);
    acosArg1 = constrain(acosArg1, -1.0, 1.0);
    acosArg2 = constrain(acosArg2, -1.0, 1.0);

    // Use the law of cosines to find Theta_2 and Theta_3
    float angle_L2_Ltarget = acos(acosArg1);
    float angle_L3_Ltarget = acos(acosArg2);
    float phi = atan2(H, D);

    // Adjusted calculations for Theta_2 and Theta_3
    Theta_2 = (phi + angle_L2_Ltarget) * (180.0 / PI); // Use angle_L2_Ltarget for Theta_2 calculation
    Theta_3 = (angle_L3_Ltarget - phi) * (180.0 / PI); // Use angle_L3_Ltarget for Theta_3 calculation, adjust formula as needed

    // Theta_4 for end effector orientation, assuming direct control over its rotation around the Z-axis
    Theta_4 = 0.0;

    // Ensure calculated angles are within the valid range for each joint
    Theta_1 = constrain(Theta_1, THETA_1_MIN, THETA_1_MAX);
    Theta_2 = constrain(Theta_2, THETA_2_MIN, THETA_2_MAX);
    Theta_3 = constrain(Theta_3, THETA_3_MIN, THETA_3_MAX);
    Theta_4 = constrain(Theta_4, THETA_4_MIN, THETA_4_MAX);
}

void setup() {
    Serial.begin(9600); // Initialize serial communication
    // Initialize stepper motors with a low speed; adjust as necessary
    stepper1.setSpeed(15);
    stepper2.setSpeed(15);
    stepper3.setSpeed(15);
    stepper4.setSpeed(15);

    // Manual calibration: Instruct the user to fully extend the arm in the positive X direction
    Serial.println("Manually extend the arm fully in the positive X direction. Press any key to confirm...");
    while (!Serial.available()) {
        // Wait for the user to confirm calibration
    }
    Serial.read(); // Clear the serial buffer after receiving input

    Serial.println("Calibration complete. The arm is now at the 0 reference position.");
}

void loop() {
    // Example usage of calculateIK function
    calculateIK(100, 100, 0); // Placeholder target position

    // Convert angles to steps
    int steps1 = (Theta_1 * stepsPerRevolution) / 360.0;
    int steps2 = (Theta_2 * stepsPerRevolution) / 360.0;
    int steps3 = (Theta_3 * stepsPerRevolution) / 360.0;
    int steps4 = (Theta_4 * stepsPerRevolution) / 360.0;

    // Move stepper motors to the calculated positions
    stepper1.step(steps1);
    stepper2.step(steps2);
    stepper3.step(steps3);
    stepper4.step(steps4);

    // Delay to allow movements to complete before the next loop iteration
    delay(2000); // Adjust delay as needed based on your application requirements

}