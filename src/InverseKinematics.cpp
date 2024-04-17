#include "InverseKinematics.h"
#include <Arduino.h>
#include "GlobalVars.h"

float Theta_1 = 0.0; // Base rotation
float Theta_2 = 90.0; // First joint
float Theta_3 = 0.0; // Second joint
float Theta_4 = 90.0; // Unused in two-link model, adjust if necessary

float currentX = 0.0;
float currentY = 0.0;
float currentZ = 495.0;

// Arm Segment Lengths (in mm)
const float L1 = 110.0; // Base to first joint
const float L2 = 255.0; // First to second joint

const float MAX_REACH = 495.0;
const float MIN_HEIGHT = 0.0, MAX_HEIGHT = 495.0;

void calculateIK(float X, float Y, float Z) {
    Serial.println("--------------------------------------------------");
    Serial.print("Received Target Coordinates: X=");
    Serial.print(X);
    Serial.print(", Y=");
    Serial.print(Y);
    Serial.print(", Z=");
    Serial.println(Z);

    // Base rotation
    Theta_1 = atan2(Y, X) * (180.0 / PI);
    Serial.print("Calculated Theta_1 (Base rotation): ");
    Serial.println(Theta_1);

    // Calculate the distance from the base to the point (X, Y)
    float r = sqrt(X * X + Y * Y);
    float dx = r;
    float dy = Z;
    // Calculate Theta_2 and Theta_3 using the provided equations
    float q2 = (acos((dx*dx+dy*dy - L1 * L1 - L2 * L2) / (2 * L1 * L2))) * -1;
    float q1 = atan2(dy, dx) + atan2(L2 * sin(q2), L1 + L2 * cos(q2));

    Theta_2 = q1 * (180.0 / PI);
    Theta_3 = (q2 * (180.0 / PI)) * -1;

    Serial.print("Calculated Theta_2: ");
    Serial.println(Theta_2);
    Serial.print("Calculated Theta_3: ");
    Serial.println(Theta_3);

    // Assuming Theta_4 is not used in this two-link model
    Theta_4 = (abs(Theta_2) + abs(Theta_3)) * -1;

    Serial.print("Calculated Theta_4: ");
    Serial.println(Theta_4);

    // Set target positions for global use
    currentX = X;
    currentY = Y;
    currentZ = Z;
}