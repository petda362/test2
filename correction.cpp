// Projectgroup 1
// Bachelors-project in Electronics Design engineering
// Alexander Riex ED4

//-------------Libraries---------------
#include "correction.h"
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

bool orth = false;          // bolean that describes if the vehicle is parallell to the walls


// ------------------------------- function ---------------------------

void clear_pin(int x)
{
  digitalWrite(x, LOW);
}

int ultraSensor(int trig, int echo)
{
  long travelTime;
  float distance;
  clear_pin(trig);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  clear_pin(trig);
  travelTime = pulseIn(echo, HIGH);
  distance = (travelTime / 2) / 29.1 * 10; // distance in mm
  return distance;
}

int real_distance(float distance, float angle)
{
  double real_dist = distance * sin(DEG_TO_RAD * angle);
  return real_dist;
}

void rotational_correction (double dist_FL, double dist_BL, double dist_FR, double dist_BR, int tolerance_angle) {
    if ((dist_BL < (dist_FL - tolerance_angle)) ||
           (dist_FR < (dist_BR - tolerance_angle)))
  {
    rotate_centered_cclkw();
    orth = false;
  }
  else if ((dist_FL < (dist_BL - tolerance_angle) ||
            (dist_FR > (dist_BR + tolerance_angle))))
  {
    rotate_centered_clkw();
    orth = false;
  }
  else if (!orth) {
    orth = true;
    rotate_stop();
  }
}

void translational_correction (double dist_FL, double dist_BL, double dist_FR, double dist_BR, int tolerance) {
    if (orth && ((dist_FL >= (dist_FR - tolerance)) &&
               (dist_FL <= (dist_FR + tolerance))))
  {
    translate_stop();
  }
  else if (orth && ((dist_FL < (dist_FR - tolerance)) || 
                    (dist_BL < (dist_BR - tolerance))))
  {
    translate_right();
  }
  else if (orth && ((dist_FL > (dist_FR + tolerance)) || 
                    (dist_BL > (dist_BR + tolerance))))
  {
    translate_left();
  }
}