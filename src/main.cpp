#include <Arduino.h>
#include <Stepper.h>


const int stepsPerRevolution = 200; // Adjust based on your stepper motor

Stepper stepper1(stepsPerRevolution, 2, 3, 4, 5); // Pins for the first joint stepper motor
Stepper stepper2(stepsPerRevolution, 6, 7, 8, 9); // Pins for the second joint stepper motor
Stepper stepper3(stepsPerRevolution, 10, 11, 12, 13); // Pins for the third joint stepper motor
Stepper stepper4(stepsPerRevolution, 14, 15, 16, 17); // Pins for the fourth joint stepper motor

float Theta_1, Theta_2, Theta_3, Theta_4;

float Pi = 3.141592653589793;
float Yoffset;
float D;
float d;
float R;
float L1 = 64.50; //height of the first link from surface to 2nd joint position
float L2 = 105.00; //length of the second link from 2nd joint to 3rd joint
float L3 = 98.50; //length of the 3rd joint to 4th joint
float L4 = 115.00; //length of the 4th joint to the tip of the arm (gripper)

float X_End_Effector; //x axis coordinate of the gripper
float Y_End_Effector; //y axis coordinate of the gripper
float Z_End_Effector; //z axis coordinate of the gripper

int pin_servo1 = 9; //joint 1 servo (yellow cable) attach to pin 9 on the arduino board
int pin_servo2 = 5; //joint 2 servo (yellow cable) attach to pin 5 on the arduino board
int pin_servo3 = 10; //joint 3 servo (yellow cable) attach to pin 10 on the arduino board
int pin_servo4 = 6; //joint 4 servo (yellow cable) attach to pin 6 on the arduino board
int pin_gripper = 3; //gripper servo (yellow cable) attach to pin 3 on the arduino board

float alpha1;
float alpha2;

// Define maximum and minimum limits for each axis
const float X_MAX = 200.0;
const float X_MIN = -200.0;
const float Y_MAX = 300.0;
const float Y_MIN = 0.0;
const float Z_MAX = 150.0;
const float Z_MIN = -50.0;

bool isWithinLimits(float x, float y, float z) {
    return x >= X_MIN && x <= X_MAX &&
           y >= Y_MIN && y <= Y_MAX &&
           z >= Z_MIN && z <= Z_MAX;
}

bool areAnglesValid(float theta1, float theta2, float theta3, float theta4) {
    // Define angle limits for each joint, adjust these based on your arm's design
    const float THETA_1_MIN = 0.0, THETA_1_MAX = 180.0;
    const float THETA_2_MIN = 0.0, THETA_2_MAX = 180.0;
    const float THETA_3_MIN = 0.0, THETA_3_MAX = 180.0;
    const float THETA_4_MIN = 0.0, THETA_4_MAX = 180.0;

    return theta1 >= THETA_1_MIN && theta1 <= THETA_1_MAX &&
           theta2 >= THETA_2_MIN && theta2 <= THETA_2_MAX &&
           theta3 >= THETA_3_MIN && theta3 <= THETA_3_MAX &&
           theta4 >= THETA_4_MIN && theta4 <= THETA_4_MAX;
}

void Rumus_IK(float X_End_Effector, float Y_End_Effector, float Z_End_Effector)
{
  if (X_End_Effector > 0 && Y_End_Effector >= L1)
  {
    D = sqrt(pow(X_End_Effector,2) + pow(Z_End_Effector,2));
    Theta_1 = (atan(Z_End_Effector/X_End_Effector))*(180.00/Pi); //theta 1
    d = D - L4;
    Yoffset = Y_End_Effector - L1;
    R = sqrt(pow(d,2) + pow(Yoffset,2));
    alpha1 = (acos(d/R))*(180.00/Pi);
    alpha2 = (acos((pow(L2,2) + pow(R,2) - pow(L3,2))/(2*L2*R)))*(180.00/Pi);
    Theta_2 = (alpha1 + alpha2); //theta 2
    Theta_3 = ((acos((pow(L2,2) + pow(L3,2) - pow(R,2))/(2*L2*L3)))*(180.00/Pi)); //theta 3
    Theta_4 = 180.00 - ((180.00 - (alpha2 + Theta_3)) - alpha1); //theta 4
  }
  else if (X_End_Effector > 0 && Y_End_Effector <= L1)
  {
    D = sqrt(pow(X_End_Effector,2) + pow(Z_End_Effector,2));
    Theta_1 = (atan(Z_End_Effector/X_End_Effector))*(180.00/Pi); //theta 1
    d = D - L4;
    Yoffset = Y_End_Effector - L1;
    R = sqrt(pow(d,2) + pow(Yoffset,2));
    alpha1 = (acos(d/R))*(180.00/Pi);
    alpha2 = (acos((pow(L2,2) + pow(R,2) - pow(L3,2))/(2*L2*R)))*(180.00/Pi);
    Theta_2 = (alpha2 - alpha1); //theta 2
    Theta_3 = ((acos((pow(L2,2) + pow(L3,2) - pow(R,2))/(2*L2*L3)))*(180.00/Pi)); //theta 3
    Theta_4 = 180.00 - ((180.00 - (alpha2 + Theta_3)) + alpha1); //theta 4
  }
  else if (X_End_Effector == 0 && Y_End_Effector >= L1)
  {
    D = sqrt(pow(X_End_Effector,2) + pow(Z_End_Effector,2));
    Theta_1 = 90.00; //theta 1
    d = D - L4;
    Yoffset = Y_End_Effector - L1;
    R = sqrt(pow(d,2) + pow(Yoffset,2));
    alpha1 = (acos(d/R))*(180.00/Pi);
    alpha2 = (acos((pow(L2,2) + pow(R,2) - pow(L3,2))/(2*L2*R)))*(180.00/Pi);
    Theta_2 = (alpha1 + alpha2); //theta 2
    Theta_3 = ((acos((pow(L2,2) + pow(L3,2) - pow(R,2))/(2*L2*L3)))*(180.00/Pi)); //theta 3
    Theta_4 = 180.00 - ((180.00 - (alpha2 + Theta_3)) - alpha1); //theta 4
  }
  else if (X_End_Effector == 0 && Y_End_Effector <= L1)
  {
    D = sqrt(pow(X_End_Effector,2) + pow(Z_End_Effector,2));
    Theta_1 = 90.00; //theta 1
    d = D - L4;
    Yoffset = Y_End_Effector - L1;
    R = sqrt(pow(d,2) + pow(Yoffset,2));
    alpha1 = (acos(d/R))*(180.00/Pi);
    alpha2 = (acos((pow(L2,2) + pow(R,2) - pow(L3,2))/(2*L2*R)))*(180.00/Pi);
    Theta_2 = (alpha2 - alpha1); //theta 2
    Theta_3 = ((acos((pow(L2,2) + pow(L3,2) - pow(R,2))/(2*L2*L3)))*(180.00/Pi)); //theta 3
    Theta_4 = 180.00 - ((180.00 - (alpha2 + Theta_3)) + alpha1); //theta 4
  }
  else if (X_End_Effector < 0 && Y_End_Effector >= L1)
  {
    D = sqrt(pow(X_End_Effector,2) + pow(Z_End_Effector,2));
    Theta_1 = 90.00 + (90.00 - abs((atan(Z_End_Effector/X_End_Effector))*(180.00/Pi))); //theta 1
    d = D - L4;
    Yoffset = Y_End_Effector - L1;
    R = sqrt(pow(d,2) + pow(Yoffset,2));
    alpha1 = (acos(d/R))*(180.00/Pi);
    alpha2 = (acos((pow(L2,2) + pow(R,2) - pow(L3,2))/(2*L2*R)))*(180.00/Pi);
    Theta_2 = (alpha1 + alpha2); //theta 2
    Theta_3 = ((acos((pow(L2,2) + pow(L3,2) - pow(R,2))/(2*L2*L3)))*(180.00/Pi)); //theta 3
    Theta_4 = 180.00 - ((180.00 - (alpha2 + Theta_3)) - alpha1); //theta 4
  }
  else if (X_End_Effector < 0 && Y_End_Effector <= L1)
  {
    D = sqrt(pow(X_End_Effector,2) + pow(Z_End_Effector,2));
    Theta_1 = 90.00 + (90.00 - abs((atan(Z_End_Effector/X_End_Effector))*(180.00/Pi))); //theta 1
    d = D - L4;
    Yoffset = Y_End_Effector - L1;
    R = sqrt(pow(d,2) + pow(Yoffset,2));
    alpha1 = (acos(d/R))*(180.00/Pi);
    alpha2 = (acos((pow(L2,2) + pow(R,2) - pow(L3,2))/(2*L2*R)))*(180.00/Pi);
    Theta_2 = (alpha2 - alpha1); //theta 2
    Theta_3 = ((acos((pow(L2,2) + pow(L3,2) - pow(R,2))/(2*L2*L3)))*(180.00/Pi)); //theta 3
    Theta_4 = 180.00 - ((180.00 - (alpha2 + Theta_3)) + alpha1); //theta 4
  }
}

void setup() {
    Serial.begin(9600); // Initialize serial communication at 9600 bits per second
    stepper1.setSpeed(60);
    stepper2.setSpeed(60);
    stepper3.setSpeed(60);
    stepper4.setSpeed(60);
}

void moveToPosition(float X_End_Effector, float Y_End_Effector, float Z_End_Effector) {
    if (!isWithinLimits(X_End_Effector, Y_End_Effector, Z_End_Effector)) {
        Serial.println("Error: Desired position is out of limits.");
        return; // Skip movement or handle error as needed
    }

    Rumus_IK(X_End_Effector, Y_End_Effector, Z_End_Effector); // Calculate joint angles based on desired position

    if (!areAnglesValid(Theta_1, Theta_2, Theta_3, Theta_4)) {
        Serial.println("Error: Calculated angles are out of bounds.");
        return; // Skip movement or handle error as needed
    }

    // Convert angles to steps (example conversion, adjust based on your setup)
    int steps1 = (Theta_1 * stepsPerRevolution) / 360;
    int steps2 = (Theta_2 * stepsPerRevolution) / 360;
    int steps3 = (Theta_3 * stepsPerRevolution) / 360;
    int steps4 = (Theta_4 * stepsPerRevolution) / 360;

    // Move stepper motors
    stepper1.step(steps1);
    stepper2.step(steps2);
    stepper3.step(steps3);
    stepper4.step(steps4);
}

void loop() {
    // Check if there is any serial input
    if (Serial.available() > 0) {
        // Read the input as a string
        String inputString = Serial.readStringUntil('\n'); // Assuming inputs are newline-separated
        // Parse the input string into X, Y, Z coordinates
        float X, Y, Z;
        int firstComma = inputString.indexOf(',');
        int secondComma = inputString.indexOf(',', firstComma + 1);
        
        if (firstComma != -1 && secondComma != -1) {
            X = inputString.substring(0, firstComma).toFloat(); // Convert to float
            Y = inputString.substring(firstComma + 1, secondComma).toFloat(); // Convert to float
            Z = inputString.substring(secondComma + 1).toFloat(); // Convert to float

            // Assuming the input is in meters and needs to be converted to the unit your system uses
            X *= 1000; // Convert meters to millimeters if necessary
            Y *= 1000; // Convert meters to millimeters if necessary
            Z *= 1000; // Convert meters to millimeters if necessary

            // Move to the new position
            moveToPosition(X, Y, Z);

            // Output the angles of all joints
            Serial.print("Theta_1: ");
            Serial.print(Theta_1);
            Serial.print(" degrees, Theta_2: ");
            Serial.print(Theta_2);
            Serial.print(" degrees, Theta_3: ");
            Serial.print(Theta_3);
            Serial.print(" degrees, Theta_4: ");
            Serial.println(Theta_4);
        } else {
            Serial.println("Error: Invalid input format. Please input as X,Y,Z in meters.");
        }
    }

    delay(100); // Short delay to allow for serial communication stability
}
