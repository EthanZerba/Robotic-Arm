#include <Arduino.h>
#include <Stepper.h>

const double L1 = 0.5; // Length of the first arm segment
const double L2 = 0.3; // Length of the second arm segment

const int stepsPerRevolution = 200; // Adjust based on your stepper motor
const double armALength = 0.5; // A-length in meters or another unit
const double armBLength = 0.3; // B-length in meters or another unit

Stepper baseStepper(stepsPerRevolution, 2, 3, 4, 5); // Pins for base stepper motor
Stepper xRotationStepper(stepsPerRevolution, 6, 7, 8, 9); // Pins for X-axis rotation
Stepper yRotationStepper(stepsPerRevolution, 10, 11, 12, 13); // Pins for Y-axis rotation
Stepper zRotationStepper(stepsPerRevolution, 14, 15, 16, 17); // Pins for Z-axis rotation

void calculateJointAngles(double x, double y, double& theta1, double& theta2) {
    double D = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2);
    theta2 = atan2(sqrt(1 - D*D), D);
    theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));
}

void setup() {
    Serial.begin(9600); // Initialize serial communication at 9600 bits per second
    baseStepper.setSpeed(60);
    xRotationStepper.setSpeed(60);
    yRotationStepper.setSpeed(60);
    zRotationStepper.setSpeed(60);
}

void loop() {
    double x = 0.5, y = 0.3; // Example values, replace with actual input
    double theta1, theta2;
    calculateJointAngles(x, y, theta1, theta2);

    theta1 = theta1 * (180.0 / M_PI);
    theta2 = theta2 * (180.0 / M_PI);

    Serial.print("Joint 1 angle: ");
    Serial.print(theta1);
    Serial.println(" degrees");
    Serial.print("Joint 2 angle: ");
    Serial.print(theta2);
    Serial.println(" degrees");

    delay(1000); // Delay for a second before the next loop iteration
}
