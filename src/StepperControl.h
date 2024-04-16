#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

void disableSteppers();
void enableSteppers();

void moveSteppersToCalculatedPositions();
void initializeSteppers();


#endif