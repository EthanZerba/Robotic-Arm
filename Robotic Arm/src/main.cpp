#include <Arduino.h>

const double L1 = 0.5; // Length of the first arm segment
const double L2 = 0.3; // Length of the second arm segment

void calculateJointAngles(double x, double y, double& theta1, double& theta2) {
    double D = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2);
    theta2 = atan2(sqrt(1 - D*D), D);
    theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));
}

void setup() {
    Serial.begin(9600); // Initialize serial communication at 9600 bits per second
}

void loop() {
    // Assuming x, y values are obtained somehow, for example from serial input
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
