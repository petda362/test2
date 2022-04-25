// Projectgroup 1
// Bachelors-project in Electronics Design engineering
// 2022

// Alexander Riex ED4
// Henrik Nilsson ED3
// Konrad Råström ED3
// Petter Danev ED5

//-------------Libraries---------------
#include "movement.h"

//---------Defining pins----------------
#define echopin_FL 40 // Forward Left sensor
#define trigpin_FL 41
#define echopin_FR 42 // Forward Right sensor
#define trigpin_FR 43
#define echopin_BL 44 // Back Left
#define trigpin_BL 45
#define echopin_BR 46 // Back Right
#define trigpin_BR 47

#define STATE 50      // STATE PIN HC05

#define FWDpin_FL 9 // FWD Forward left
#define BWDpin_FL 8 // BWD
#define FWDpin_FR 4 // FWD Forward Right
#define BWDpin_FR 5 // BWD
#define FWDpin_BL 7 // FWD Backward Left
#define BWDpin_BL 6 // BWD
#define FWDpin_BR 10 // FWD Backward Right
#define BWDpin_BR 11 // BWD


// --------------------- changeable variables-----------------

float multiplier_FL = 0.989;         // Multipliers for seperate wheels, for adjusting motor speed (PWM * multiplier)
float multiplier_FR = 1;
float multiplier_BL = 0.989;
float multiplier_BR = 1;
float multiplier_rotation = 1;    // multiplier for rotation, for adjusting motor speed whilst rotating (PWM * multiplier)
int current_dir;


// ------------------------------- function ---------------------------

void diagonal_FW_right(int PWM)
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

void diagonal_FW_left(int PWM)
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

void translate_right(int PWM)
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

void translate_left(int PWM)
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

void translate_FWD_pid(int PWM, float pid_output)
{
  analogWrite(FWDpin_FL, (0 * multiplier_FL));
  analogWrite(BWDpin_FL, (PWM - pid_output));

  analogWrite(FWDpin_FR, (0 * multiplier_FR));
  analogWrite(BWDpin_FR, (PWM + pid_output));

  analogWrite(FWDpin_BL, (0 * multiplier_BL));
  analogWrite(BWDpin_BL, (PWM - pid_output));

  analogWrite(FWDpin_BR, (0 * multiplier_BR));
  analogWrite(BWDpin_BR, (PWM + pid_output));
    current_dir = 1;                                      // Set current direction as translate FWD
}

void translate_FWD(int PWM)
{
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

void translate_BWD(int PWM)
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

void rotate_centered_clkw(int PWM)
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

void rotate_centered_cclkw(int PWM)
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

void rotate_clkw_rear(int PWM)
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

void rotate_cclkw_rear(int PWM)
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

void rotate_clkw_front(int PWM)
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

void rotate_cclkw_front(int PWM)
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

void quickbrake(int PWM) {
    unsigned int brake_delay = (PWM) / 3;

    switch(current_dir) {           // Every set of movements has a number correlated to it
        case 1:                     // case = 1 current dir forward
            translate_BWD(PWM);
            delay(brake_delay);
            translate_stop();

        break;
        case 2:                     // case = 2 current dir backward
            translate_FWD(PWM);
            delay(brake_delay);
            translate_stop();
        break;
        case 3:                     // case = 3 current dir right
            translate_left(PWM);
            delay(brake_delay);
            translate_stop();
        break;
        case 4:                     // case = 4 current dir left
            translate_right(PWM);
            delay(brake_delay);
            translate_stop();

        break;
        case 5:                     // case = 5 current dir rotate clkw
            rotate_centered_cclkw(PWM);
            delay(brake_delay);
            rotate_stop();

        break;
        case 6:                     // case = 6 current dir rotate c-clkw
            rotate_centered_clkw(PWM);
            delay(brake_delay);
            rotate_stop();

        break;
        case 7:                     // case = 7 current dir rotate clockwise on the rear axis
            rotate_cclkw_rear(PWM);    
            delay(brake_delay);
            rotate_stop();
        break;
        case 8:                     // case = 8 current dir rotate counter clockwise on the rear axis
            rotate_clkw_rear(PWM);     
            delay(brake_delay);
            rotate_stop();

        break;
        case 9:                     // case = 9 current dir rotate clockwise on the front axis
            rotate_cclkw_front(PWM);
            delay(brake_delay);
            rotate_stop();

        break;
        case 10:                    // case 10 current dir rotate counter clockwise on the rear axis
            rotate_clkw_front(PWM);
            delay(brake_delay);
            rotate_stop();
        break;
    }

}