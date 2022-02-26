// Projectgroup 1
// Bachelors-project in Electronics Design engineering
// Alexander Riex ED4

//-------------Libraries---------------
#include "movement.h"


//---------Defining pins----------------
#define echopin_FL 50 // Forward Left ultrasonic sensor
#define trigpin_FL 52
#define echopin_FR 47 // Forward Right ultrasonic sensor
#define trigpin_FR 49
#define echopin_BL 51 // Back Left ultrasonic sensor
#define trigpin_BL 53
#define echopin_BR 46 // Back Right ultrasonic sensor
#define trigpin_BR 48

#define FWDpin_FL 9 // FWD Forward left
#define BWDpin_FL 8 // BWD -||-
#define FWDpin_FR 7 // FWD Forward Right
#define BWDpin_FR 6 // BWD -||-
#define FWDpin_BL 5 // FWD Backward Left
#define BWDpin_BL 4 // BWD -||-
#define FWDpin_BR 3 // FWD Backward Right
#define BWDpin_BR 2 // BWD -||-


// --------------------- changeable variables-----------------
int PWM = 100;                      // Base PWM before modifiers. from 0 to 255
float multiplier_FL = 0.90;         // Multipliers for seperate wheels, for adjusting motor speed (PWM * multiplier)
float multiplier_FR = 0.90;
float multiplier_BL = 1.0;
float multiplier_BR = 0.90;
float multiplier_rotation = 0.8;    // multiplier for rotation, for adjusting motor speed whilst rotating (PWM * multiplier)
int current_dir;


// ------------------------------- function ---------------------------

void diagonal_FW_right()
{
  analogWrite(FWDpin_FL, (PWM * multiplier_FL));
  analogWrite(BWDpin_FL, (0 * multiplier_FL));

  analogWrite(FWDpin_FR, (0 * multiplier_FR));
  analogWrite(BWDpin_FR, (0 * multiplier_FR));

  analogWrite(FWDpin_BL, (0 * multiplier_BL));
  analogWrite(BWDpin_BL, (0 * multiplier_BL));

  analogWrite(FWDpin_BR, (PWM * multiplier_BR));
  analogWrite(BWDpin_BR, (0 * multiplier_BR));
}

void diagonal_FW_left()
{
  analogWrite(FWDpin_FL, (0 * multiplier_FL));
  analogWrite(BWDpin_FL, (0 * multiplier_FL));

  analogWrite(FWDpin_FR, (PWM * multiplier_FR));
  analogWrite(BWDpin_FR, (0 * multiplier_FR));

  analogWrite(FWDpin_BL, (PWM * multiplier_BL));
  analogWrite(BWDpin_BL, (0 * multiplier_BL));

  analogWrite(FWDpin_BR, (0 * multiplier_BR));
  analogWrite(BWDpin_BR, (0 * multiplier_BR));
}

void translate_right()
{
  Serial.println("move right");

  analogWrite(FWDpin_FL, (0 * multiplier_FL));
  analogWrite(BWDpin_FL, (PWM * multiplier_FL));

  analogWrite(FWDpin_FR, (PWM * multiplier_FR));
  analogWrite(BWDpin_FR, (0 * multiplier_FR));

  analogWrite(FWDpin_BL, (PWM * multiplier_BL));
  analogWrite(BWDpin_BL, (0 * multiplier_BL));

  analogWrite(FWDpin_BR, (0 * multiplier_BR));
  analogWrite(BWDpin_BR, (PWM * multiplier_BR));

  current_dir = 3;                                      // Set current direction as translate right

}

void translate_left()
{

  Serial.println("move left");

  analogWrite(FWDpin_FL, (PWM * multiplier_FL));
  analogWrite(BWDpin_FL, (0 * multiplier_FL));

  analogWrite(FWDpin_FR, (0 * multiplier_FR));
  analogWrite(BWDpin_FR, (PWM * multiplier_FR));

  analogWrite(FWDpin_BL, (0 * multiplier_BL));
  analogWrite(BWDpin_BL, (PWM * multiplier_BL));

  analogWrite(FWDpin_BR, (PWM * multiplier_BR));
  analogWrite(BWDpin_BR, (0 * multiplier_BR));

  current_dir = 4;                                      // Set current direction as translate left
}

void translate_stop()
{
  Serial.println("Centered");

  analogWrite(FWDpin_FL, 0);
  analogWrite(BWDpin_FL, 0);

  analogWrite(FWDpin_FR, 0);
  analogWrite(BWDpin_FR, 0);

  analogWrite(FWDpin_BL, 0);
  analogWrite(BWDpin_BL, 0);

  analogWrite(FWDpin_BR, 0);
  analogWrite(BWDpin_BR, 0);
}

void rotate_stop()
{
  Serial.println("ortogonal");

  analogWrite(FWDpin_FL, 0);
  analogWrite(BWDpin_FL, 0);

  analogWrite(FWDpin_FR, 0);
  analogWrite(BWDpin_FR, 0);

  analogWrite(FWDpin_BL, 0);
  analogWrite(BWDpin_BL, 0);

  analogWrite(FWDpin_BR, 0);
  analogWrite(BWDpin_BR, 0);
}

void translate_FWD()
{
  Serial.println("Forward");
  analogWrite(FWDpin_FL, (0 * multiplier_FL));
  analogWrite(BWDpin_FL, (PWM * multiplier_FL));

  analogWrite(FWDpin_FR, (0 * multiplier_FR));
  analogWrite(BWDpin_FR, (PWM * multiplier_FR));

  analogWrite(FWDpin_BL, (0 * multiplier_BL));
  analogWrite(BWDpin_BL, (PWM * multiplier_BL));

  analogWrite(FWDpin_BR, (0 * multiplier_BR));
  analogWrite(BWDpin_BR, (PWM * multiplier_BR));

    current_dir = 1;                                      // Set current direction as translate FWD
}

void translate_BWD()
{
  Serial.println("Backward");
  analogWrite(FWDpin_FL, (PWM * multiplier_FL));
  analogWrite(BWDpin_FL, (0 * multiplier_FL));

  analogWrite(FWDpin_FR, (PWM * multiplier_FR));
  analogWrite(BWDpin_FR, (0 * multiplier_FR));

  analogWrite(FWDpin_BL, (PWM * multiplier_BL));
  analogWrite(BWDpin_BL, (0 * multiplier_BL));

  analogWrite(FWDpin_BR, (PWM * multiplier_BR));
  analogWrite(BWDpin_BR, (0 * multiplier_BR));

    current_dir = 2;                                      // Set current direction as translate BWD
}

void rotate_centered_clkw()
{
  Serial.println("rotate clockwise");
  analogWrite(FWDpin_FL, (0 * multiplier_FL * multiplier_rotation));
  analogWrite(BWDpin_FL, (PWM * multiplier_FL * multiplier_rotation));

  analogWrite(FWDpin_FR, (PWM * multiplier_FR * multiplier_rotation));
  analogWrite(BWDpin_FR, (0 * multiplier_FR * multiplier_rotation));

  analogWrite(FWDpin_BL, (0 * multiplier_BL * multiplier_rotation));
  analogWrite(BWDpin_BL, (PWM * multiplier_BL * multiplier_rotation));

  analogWrite(FWDpin_BR, (PWM * multiplier_BR * multiplier_rotation));
  analogWrite(BWDpin_BR, (0 * multiplier_BR * multiplier_rotation));

  current_dir = 5;                                      // Set current direction as rotate clockwise
}

void rotate_centered_cclkw()
{
  Serial.println("rotate counter-clockwise");
  analogWrite(FWDpin_FL, (PWM * multiplier_FL * multiplier_rotation));
  analogWrite(BWDpin_FL, (0 * multiplier_FL * multiplier_rotation));

  analogWrite(FWDpin_FR, (0 * multiplier_FR * multiplier_rotation));
  analogWrite(BWDpin_FR, (PWM * multiplier_FR * multiplier_rotation));

  analogWrite(FWDpin_BL, (PWM * multiplier_BL * multiplier_rotation));
  analogWrite(BWDpin_BL, (0 * multiplier_BL * multiplier_rotation));

  analogWrite(FWDpin_BR, (0 * multiplier_BR * multiplier_rotation));
  analogWrite(BWDpin_BR, (PWM * multiplier_BR * multiplier_rotation));

  current_dir = 6;                                      // Set current direction as rotate counter clockwise
}

void rotate_clkw_rear()
{
  analogWrite(FWDpin_FL, (PWM * multiplier_FL));
  analogWrite(BWDpin_FL, (0 * multiplier_FL));

  analogWrite(FWDpin_FR, (0 * multiplier_FR));
  analogWrite(BWDpin_FR, (PWM * multiplier_FR));

  analogWrite(FWDpin_BL, (0 * multiplier_BL));
  analogWrite(BWDpin_BL, (0 * multiplier_BL));

  analogWrite(FWDpin_BR, (0 * multiplier_BR));
  analogWrite(BWDpin_BR, (0 * multiplier_BR));
  
  current_dir = 7;                                      // Set current direction as rotate clockwise on the rear axis
}

void rotate_cclkw_rear()
{
  analogWrite(FWDpin_FL, (0 * multiplier_FL));
  analogWrite(BWDpin_FL, (PWM * multiplier_FL));

  analogWrite(FWDpin_FR, (PWM * multiplier_FR));
  analogWrite(BWDpin_FR, (0 * multiplier_FR));

  analogWrite(FWDpin_BL, (0 * multiplier_BL));
  analogWrite(BWDpin_BL, (0 * multiplier_BL));

  analogWrite(FWDpin_BR, (0 * multiplier_BR));
  analogWrite(BWDpin_BR, (0 * multiplier_BR));

  current_dir = 8;                                      // Set current direction as rotate counter clockwise on the rear axis
}

void rotate_clkw_front()
{
  analogWrite(FWDpin_FL, (0 * multiplier_FL));
  analogWrite(BWDpin_FL, (0 * multiplier_FL));

  analogWrite(FWDpin_FR, (0 * multiplier_FR));
  analogWrite(BWDpin_FR, (0 * multiplier_FR));

  analogWrite(FWDpin_BL, (PWM * multiplier_BL));
  analogWrite(BWDpin_BL, (0 * multiplier_BL));

  analogWrite(FWDpin_BR, (0 * multiplier_BR));
  analogWrite(BWDpin_BR, (PWM * multiplier_BR));

  current_dir = 9;                                      // Set current direction as rotate clockwise on the front axis
}

void rotate_cclkw_front()
{
  analogWrite(FWDpin_FL, (0 * multiplier_FL));
  analogWrite(BWDpin_FL, (0 * multiplier_FL));

  analogWrite(FWDpin_FR, (0 * multiplier_FR));
  analogWrite(BWDpin_FR, (0 * multiplier_FR));

  analogWrite(FWDpin_BL, (0 * multiplier_BL));
  analogWrite(BWDpin_BL, (PWM * multiplier_BL));

  analogWrite(FWDpin_BR, (PWM * multiplier_BR));
  analogWrite(BWDpin_BR, (0 * multiplier_BR));

  current_dir = 10;                                      // Set current direction as rotate counter clockwise on the rear axis
}

void quickbrake() {
    unsigned int delay = PWM / 10;

    switch(current_dir) {           // Every set of movements has a number correlated to it
        case 1:                     // case = 1 current dir forward
            translate_BWD();
            delayMicroseconds(delay);
            translate_stop();

        break;
        case 2:                     // case = 2 current dir backward
            translate_FWD();
            delayMicroseconds(delay);
            translate_stop();
        break;
        case 3:                     // case = 3 current dir right
            translate_left();
            delayMicroseconds(delay);
            translate_stop();
        break;
        case 4:                     // case = 4 current dir left
            translate_right();
            delayMicroseconds(delay);
            translate_stop();

        break;
        case 5:                     // case = 5 current dir rotate clkw
            rotate_centered_cclkw();
            delayMicroseconds(delay);
            rotate_stop();

        break;
        case 6:                     // case = 6 current dir rotate c-clkw
            rotate_centered_clkw();
            delayMicroseconds(delay);
            rotate_stop();

        break;
        case 7:                     // case = 7 current dir rotate clockwise on the rear axis
            rotate_cclkw_rear();    
            delayMicroseconds(delay);
            rotate_stop();
        break;
        case 8:                     // case = 8 current dir rotate counter clockwise on the rear axis
            rotate_clkw_rear();     
            delayMicroseconds(delay);
            rotate_stop();

        break;
        case 9:                     // case = 9 current dir rotate clockwise on the front axis
            rotate_cclkw_front();
            delayMicroseconds(delay);
            rotate_stop();

        break;
        case 10:                    // case 10 current dir rotate counter clockwise on the rear axis
            rotate_clkw_front();
            delayMicroseconds(delay);
            rotate_stop();
        break;
    }

}