#include <Arduino.h>
#include <AccelStepper.h>
#include "InverseKinematics.h"
#include "SerialComm.h"
#include "StepperControl.h"
#include "GlobalVars.h"


void setup() {
    Serial.begin(9600);
    initializeSteppers();

    Serial.println("Calibration assumed at 495,0,0. Ready for commands.");
    Serial.println("Enter coordinates in the format X,Y,Z:");
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