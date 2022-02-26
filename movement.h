#ifndef movement_h
#define movement_h
#include <Arduino.h>
#include <math.h>

void diagonal_FW_right();                         // Vehicle translates diagonally forward to the right

void diagonal_FW_left();                          // Vehicle translates diagonally forward to the right

void translate_right();                           // vehicle translates to the right

void translate_left();                            // Vehicle translates to the left

void translate_stop();                            // Set all pwm signals to 0, stop all motors

void rotate_stop();                               // Currently same as translate_stop ^

void translate_FWD();                             // Vehicle drives forwards

void translate_BWD();                             // Vehicle drives backwards

void rotate_centered_clkw();                      // Rotates vehicle clockwise from the center axis

void rotate_centered_cclkw();                     // Rotates vehicle counterclockwise from the center axis

void rotate_clkw_rear();                          // Rotates the vehichle clockwise centered on the back axis (centered)

void rotate_cclkw_rear();                         // Rotates the vehichle counter clockwise centered on the back axis (centered)

void rotate_clkw_front();                         // Rotates the vehichle clockwise centered on the front axis (centered)

void rotate_cclkw_front();                        // Rotates the vehichle counter clockwise centered on the front axis (centered)

void quickbrake(int PWM);                         // Forces an invers movement against the current direction of travel, causing a quick stop instead of rolling stop.

#endif