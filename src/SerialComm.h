#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H
#include "Arduino.h"

void handleSerialCommunication();
void handleSerialCommunication();
void resetSteppersForCalibration();
void processMultipleCoordinates(int count);
void readSerialInput();
void processSerialInput(const String& inputBuffer);
void promptUserForInput();
bool parseInput(const String& input, float& X, float& Y, float& Z);

#endif