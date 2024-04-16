#ifndef GLOBAL_H
#define GLOBAL_H

extern float targetX, targetY, targetZ;
extern float currentX, currentY, currentZ;

// Calculated Angles
extern float Theta_1, Theta_2, Theta_3, Theta_4; // Variables to store the calculated joint angles
extern float currentTheta1, currentTheta2, currentTheta3, currentTheta4; // Variables to store the current joint angles
extern long currentPositionStepper1, currentPositionStepper2, currentPositionStepper3, currentPositionStepper4;

extern const float L1, L2, L3, L4;

// Maximum reach and height (in mm)
extern const float MAX_REACH; // Maximum horizontal reach
extern const float MIN_HEIGHT, MAX_HEIGHT; // Minimum and maximum height

#endif