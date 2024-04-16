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
    handleSerialCommunication();

}