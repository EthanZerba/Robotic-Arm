#include "InverseKinematics.h"
#include <Arduino.h>
#include "GlobalVars.h"

float Theta_1, Theta_2, Theta_3, Theta_4; // Declare as extern if used across multiple files
float targetX, targetY, targetZ;


float currentX = 495.0;
float currentY = 0.0;
float currentZ = 0.0;

// Arm Segment Lengths (in mm)
const float L1 = 110.0; // Base to first joint
const float L2 = 255.0; // First to second joint
const float L3 = 240.0; // Second to third joint
const float L4 = 0.0;  // Third joint to end effector

const float MAX_REACH = 495.0;
const float MIN_HEIGHT = 0.0, MAX_HEIGHT = 495.0;

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