#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H
#include "Arduino.h"

bool parseInput(const String& input, float& X, float& Y, float& Z);

#endif