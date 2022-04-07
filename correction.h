#ifndef correction_h
#define correction_h
#include <Arduino.h>
#include <math.h>


// ------------Function declaration--------------------------------

void clear_pin(int x);                            // Set digital pin low

int ultraSensor(int trig, int echo);              // Perform measurements with the ultrasonic sensor

int real_distance(float distance, float angle);   // Convert the distance recieved from sensor into orthogonal distance based on angle (trigonometry)


void rotational_correction (double dist_FL, double dist_BL, double dist_FR, double dist_BR, int tolerance, int PWM);

void translational_correction (double dist_FL, double dist_BL, double dist_FR, double dist_BR, int tolerance, int PWM);


#endif