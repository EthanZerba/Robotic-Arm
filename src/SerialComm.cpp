#include "SerialComm.h"
#include "Arduino.h"
#include "SerialComm.h"
#include "GlobalVars.h"
#include "InverseKinematics.h"
#include "StepperControl.h"
#include <queue>
#include <string>

static String inputBuffer = "";  // Buffer to store input characters
static bool isReadyForNewInput = true;  // Flag to indicate readiness for new input
static bool expectingMultipleCoordinates = false;  // Flag to check if multiple coordinates are expecte

std::queue<std::string> coordinateQueue;

void resetSteppersForCalibration() {
    disableSteppers();  // Function to disable steppers, defined in StepperControl.cpp
    Serial.println("Steppers disabled. Move to calibration position and press 'c' to continue.");
}

std::vector<String> splitString(const String& str, char delim) {
    std::vector<String> tokens;
    int start = 0;
    int end = str.indexOf(delim);
    while (end != -1) {
        tokens.push_back(str.substring(start, end));
        start = end + 1;
        end = str.indexOf(delim, start);
    }
    tokens.push_back(str.substring(start));  // Add the last token
    return tokens;
}

void processMultipleCoordinates(const String& input) {
    std::vector<String> coordinates = splitString(input, ' ');  // Split the input string by spaces
    for (String coord : coordinates) {
        float X, Y, Z;
        if (parseInput(coord, X, Y, Z)) {
            calculateIK(X, Y, Z);
            moveSteppersToCalculatedPositions();
        }
    }
    isReadyForNewInput = true;  // Allow new input after processing
}

void promptUserForInput() {
    Serial.print("Current Position: X=");
    Serial.print(currentX);
    Serial.print(", Y=");
    Serial.print(currentY);
    Serial.print(", Z=");
    Serial.println(currentZ);
    Serial.println("Enter reset to calibrate or multi for multiple coordinates. or Enter coordinates in the format X,Y,Z:");
    isReadyForNewInput = false;  // Reset flag until next input is processed
}

void handleSerialCommunication() {
    if (isReadyForNewInput) {
        promptUserForInput();
        isReadyForNewInput = false;  // Reset flag until next input is processed
    }

    readSerialInput();  // This function will handle the reading and processing of serial input
}

void handleCoordinateInput(const String& input) {
    if (expectingMultipleCoordinates) {
        processMultipleCoordinates(input);
        expectingMultipleCoordinates = false;  // Reset flag after processing
    } else {
        float X, Y, Z;
        if (parseInput(input, X, Y, Z)) {
            calculateIK(X, Y, Z);
            moveSteppersToCalculatedPositions();
            isReadyForNewInput = true;  // Set flag to true as the robot is ready for new input
        } else {
            Serial.println("Invalid input. Please enter coordinates in the format X,Y,Z.");
            isReadyForNewInput = true;  // Allow retrying input
        }
    }
}
void processCommand(const String& input) {
    if (input == "reset") {
        resetSteppersForCalibration();
    } else if (input == "multi") {
        expectingMultipleCoordinates = true;  // Set flag to true
        Serial.println("Enter coordinates separated by spaces:");
    } else {
        handleCoordinateInput(input);
    }
}


void readSerialInput() {
    while (Serial.available() > 0) {
        char inChar = (char)Serial.read();  // Read the incoming character
        Serial.print(inChar);  // Echo the character back to the terminal

        if (inChar == '\r' || inChar == '\n') {
            if (inputBuffer.length() > 0) {  // Check if there's something to process
                Serial.println();  // Move to a new line
                processCommand(inputBuffer);
                inputBuffer = "";  // Clear the buffer after processing
            }
        } else if (inChar == '\b' && inputBuffer.length() > 0) {
            inputBuffer.remove(inputBuffer.length() - 1);  // Remove last character from buffer
            Serial.print("\b \b");  // Erase the last character on the terminal
        } else {
            inputBuffer += inChar;  // Add character to buffer
        }
    }
}

bool parseInput(const String& input, float& X, float& Y, float& Z) {
    int commaIndex1 = input.indexOf(',');
    int commaIndex2 = input.indexOf(',', commaIndex1 + 1);
    if (commaIndex1 == -1 || commaIndex2 == -1) {
        Serial.println("Error: Incorrect format, commas missing.");
        return false;
    }

    X = input.substring(0, commaIndex1).toFloat();
    Y = input.substring(commaIndex1 + 1, commaIndex2).toFloat();
    Z = input.substring(commaIndex2 + 1).toFloat();
    return true;
}
