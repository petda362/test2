// Projectgroup 1
// Bachelors-project in Electronics Design engineering
// 2022

// Alexander Riex ED4
// Henrik Nilsson ED3
// Konrad Råström ED3
// Petter Danev ED5

//-------------Libraries---------------
#include <Arduino.h>
#include <math.h>
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
#define buzzer_pin 30 // pin for audio buzzer

#define FWDpin_FL 9 // FWD Forward left
#define BWDpin_FL 8 // BWD
#define FWDpin_FR 4 // FWD Forward Right
#define BWDpin_FR 5 // BWD
#define FWDpin_BL 7 // FWD Backward LefB
#define BWDpin_BL 6 // BWD
#define FWDpin_BR 10 // FWD Backward Right
#define BWDpin_BR 11 // BWD


// --------------------- variables-------------
bool orth = false;
bool centered = false;

// ------------Functions------------------
void clear_pin(int x)
{
  digitalWrite(x, LOW);
}

int ultraSensor(int trig, int echo)
{
  long travelTime;
  float distance;
  float sum_distances = 0;
  float avg_distance;
  int num_measurements = 1;//--------------------------------------------
  
  for (int i = 0; i < num_measurements; i = i + 1) {
    clear_pin(trig);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    clear_pin(trig);
    travelTime = pulseIn(echo, HIGH);
    distance = (travelTime / 2) / 29.1 * 10; // distance in mm
    sum_distances = sum_distances + distance;
  }
  avg_distance = sum_distances / num_measurements;
  return avg_distance;
}


int real_distance(float distance, float angle)
{
  double real_dist = distance * sin(DEG_TO_RAD * angle);
  return real_dist;
}


bool rotational_correction (double dist_FL, double dist_BL, double dist_FR, double dist_BR, int tolerance_angle, int PWM) {
   
    if ((dist_BL < (dist_FL - tolerance_angle)) ||
           (dist_FR < (dist_BR - tolerance_angle)))
  {
    rotate_centered_cclkw(PWM+5);
    orth = false;
  }
  else if ((dist_FL < (dist_BL - tolerance_angle) ||
            (dist_FR > (dist_BR + tolerance_angle))))
  {
    rotate_centered_clkw(PWM+5);
    orth = false;
  }
  else if (!orth) {
    orth = true;
    quickbrake(PWM);
    // rotate_stop();
  }
  return orth;
}

void translational_correction (double dist_FL, double dist_BL, double dist_FR, double dist_BR, int tolerance, int PWM) {

    if (orth && ((dist_FL >= (dist_FR - tolerance)) &&
               (dist_FL <= (dist_FR + tolerance))))
  {
    translate_stop();
    centered = true;
  
  }
  else if (orth && ((dist_FL < (dist_FR - tolerance)) || 
                    (dist_BL < (dist_BR - tolerance))))
  {
    translate_right(PWM);
  }
  else if (orth && ((dist_FL > (dist_FR + tolerance)) || 
                    (dist_BL > (dist_BR + tolerance))))
  {
    translate_left(PWM);
  }
}

void total_correction(int tolerance_angle, int tolerance, int PWM, float angle) {
  double dist_FL;
  double dist_FR;
  double dist_BL;
  double dist_BR;
  while(!centered) {
    dist_FL = real_distance(ultraSensor(trigpin_FL, echopin_FL), angle);
    dist_FR = real_distance(ultraSensor(trigpin_FR, echopin_FR), angle);
    dist_BL = real_distance(ultraSensor(trigpin_BL, echopin_BL), angle);
    dist_BR = real_distance(ultraSensor(trigpin_BR, echopin_BR), angle);
    /*
    Serial.println(dist_FL);
    Serial.print(dist_FR);
    Serial.print(dist_BL);
    Serial.print(dist_BR);
    */
   if (orth == false){
    orth = rotational_correction(dist_FL, dist_BL, dist_FR, dist_BR, tolerance_angle, PWM-3);
   }
   else if (centered == false){ 
    translational_correction(dist_FL, dist_BL, dist_FR, dist_BR, tolerance, PWM-3);
    if(centered){
    orth = false;
    }
   }
  }
  orth = false;
  centered = false;
}

