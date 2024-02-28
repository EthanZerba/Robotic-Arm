#include <Arduino.h>
#include <Servo.h>

const double L1 = 0.5; // Length of the first arm segment
const double L2 = 0.25; // Length of the second arm segment

Servo servo1; // Declare the first servo
Servo servo2; // Declare the second servo

void calculateJointAngles(double x, double y, double& theta1, double& theta2) {
    double distance = sqrt(x*x + y*y);
    if (distance > L1 + L2 || distance < fabs(L1 - L2)) {
        Serial.println("Error: Target out of reach.");
        theta1 = theta2 = NAN; // Set angles to NaN to indicate error
        return;
    }

    double D = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2);
    if (D < -1.0 || D > 1.0) {
        Serial.print("Error: D out of range: ");
        Serial.println(D);
        theta1 = theta2 = NAN;
        return;
    }
    theta2 = atan2(sqrt(1 - D*D), D);
    theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));
}

void setup() {
    Serial.begin(9600); // Initialize serial communication at 9600 bits per second
    servo1.attach(9); // Attach the first servo to pin 9
    servo2.attach(10); // Attach the second servo to pin 10
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB
    }
    Serial.println("Enter X and Y values separated by a space:");
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n'); // Read the incoming data
        float x, y;
        int firstSpace = input.indexOf(' ');
        if (firstSpace != -1) {
            x = input.substring(0, firstSpace).toFloat();
            y = input.substring(firstSpace + 1).toFloat();

            // Check if x or y is not a number (NaN)
            if (isnan(x) || isnan(y)) {
                Serial.println("Invalid input. Please enter valid X and Y values.");
                return; // Skip the rest of the loop iteration
            }

            double theta1, theta2;
            calculateJointAngles(x, y, theta1, theta2);

            // Convert radians to degrees and check for NaN
            theta1 = theta1 * (180.0 / M_PI);
            theta2 = theta2 * (180.0 / M_PI);

            if (isnan(theta1) || isnan(theta2)) {
                Serial.println("Calculated angles are invalid.");
                return; // Skip moving the servos if angles are NaN
            }

            Serial.print("Joint 1 angle: ");
            Serial.print(theta1);
            Serial.println(" degrees");
            Serial.print("Joint 2 angle: ");
            Serial.print(theta2);
            Serial.println(" degrees");

            servo1.write(int(theta1)); // Move the first servo to theta1 angle
            servo2.write(int(theta2)); // Move the second servo to theta2 angle
        } else {
            Serial.println("Invalid input. Please enter X and Y values separated by a space.");
        }
    }
    delay(100); // Short delay to allow for serial communication
}
