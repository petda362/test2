#ifndef movement_h
#define movement_h
#include <Arduino.h>
#include <math.h>

void translate_right();                           // vehicle translates to the right

void translate_left();                            // Vehicle translates to the left

void translate_stop();                            // Set all pwm signals to 0, stop all motors

void rotate_stop();                               // Currently same as translate_stop ^

void translate_FWD();                             // Vehicle drives forwards

void translate_BWD();                             // Vehicle drives backwards

void rotate_centered_clkw();                      // Rotates vehicle clockwise from the center axis

void rotate_centered_cclkw();                     // Rotates vehicle counterclockwise from the center axis

void quickbrake(int PWM);



#endif