#ifndef correction_h
#define correction_h
#include <Arduino.h>
#include <math.h>


// ------------Function declaration--------------------------------

void clear_pin(int x);                            // Set digital pin low

int ultraSensor(int trig, int echo);              // Perform measurements with the ultrasonic sensor

int real_distance(float distance, float angle);   // Convert the distance recieved from sensor into orthogonal distance based on angle (trigonometry)

void translate_right();                           // vehicle translates to the right

void translate_left();                            // Vehicle translates to the left

void translate_stop();                            // Set all pwm signals to 0, stop all motors

void rotate_stop();                               // Currently same as translate_stop ^

void translate_FWD();                             // Vehicle drives forwards

void translate_BWD();                             // Vehicle drives backwards

void rotate_centered_clkw();                      // Rotates vehicle clockwise from the center axis

void rotate_centered_cclkw();                     // Rotates vehicle counterclockwise from the center axis

void rotational_correction (double dist_FL, double dist_BL, double dist_FR, double dist_BR, int tolerance);

void translational_correction (double dist_FL, double dist_BL, double dist_FR, double dist_BR, int tolerance);

#endif