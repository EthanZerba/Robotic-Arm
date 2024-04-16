#ifndef GLOBAL_H
#define GLOBAL_H

extern float currentX = 495.0;
extern float currentY = 0.0;
extern float currentZ = 0.0;

extern float targetX, targetY, targetZ;

// Arm Segment Lengths (in mm)
extern const float L1 = 110.0; // Base to first joint
extern const float L2 = 255.0; // First to second joint
extern const float L3 = 240.0; // Second to third joint
extern const float L4 = 0.0;  // Third joint to end effector

// Joint Angle Limits (in degrees)
extern const float THETA_1_MIN = 0.0, THETA_1_MAX = 180.0; // Limits for joint 1
extern const float THETA_2_MIN = -45.0, THETA_2_MAX = 225.0; // Limits for joint 2
extern const float THETA_3_MIN = -30.0, THETA_3_MAX = 190.0; // Limits for joint 3
extern const float THETA_4_MIN = 0.0, THETA_4_MAX = 360.0; // Limits for joint 4

// Maximum reach and height (in mm)
extern const float MAX_REACH = 495.0; // Maximum horizontal reach
extern const float MIN_HEIGHT = 0.0, MAX_HEIGHT = 495.0; // Minimum and maximum height

#endif