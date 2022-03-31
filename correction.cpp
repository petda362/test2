// Projectgroup 1
// Bachelors-project in Electronics Design engineering
// Alexander Riex ED4

//-------------Libraries---------------
#include "correction.h"

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

void translate_right()
{
  Serial.println("translate right");

  analogWrite(FWDpin_FL, (0 * multiplier_FL));
  analogWrite(BWDpin_FL, (PWM * multiplier_FL));

  analogWrite(FWDpin_FR, (PWM * multiplier_FR));
  analogWrite(BWDpin_FR, (0 * multiplier_FR));

  analogWrite(FWDpin_BL, (PWM * multiplier_BL));
  analogWrite(BWDpin_BL, (0 * multiplier_BL));

  analogWrite(FWDpin_BR, (0 * multiplier_BR));
  analogWrite(BWDpin_BR, (PWM * multiplier_BR));
}

void translate_left()
{

  Serial.println("translate left");

  analogWrite(FWDpin_FL, (PWM * multiplier_FL));
  analogWrite(BWDpin_FL, (0 * multiplier_FL));

  analogWrite(FWDpin_FR, (0 * multiplier_FR));
  analogWrite(BWDpin_FR, (PWM * multiplier_FR));

  analogWrite(FWDpin_BL, (0 * multiplier_BL));
  analogWrite(BWDpin_BL, (PWM * multiplier_BL));

  analogWrite(FWDpin_BR, (PWM * multiplier_BR));
  analogWrite(BWDpin_BR, (0 * multiplier_BR));
}

void translate_stop()
{
  Serial.println("centerd (translate stop)");

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
  Serial.println("ortogonal (rotate stop)");

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
  Serial.println("Move forward");
  analogWrite(FWDpin_FL, (0 * multiplier_FL));
  analogWrite(BWDpin_FL, (PWM * multiplier_FL));

  analogWrite(FWDpin_FR, (0 * multiplier_FR));
  analogWrite(BWDpin_FR, (PWM * multiplier_FR));

  analogWrite(FWDpin_BL, (0 * multiplier_BL));
  analogWrite(BWDpin_BL, (PWM * multiplier_BL));

  analogWrite(FWDpin_BR, (0 * multiplier_BR));
  analogWrite(BWDpin_BR, (PWM * multiplier_BR));
}

void translate_BWD()
{
  Serial.println("Move backward");
  analogWrite(FWDpin_FL, (PWM * multiplier_FL));
  analogWrite(BWDpin_FL, (0 * multiplier_FL));

  analogWrite(FWDpin_FR, (PWM * multiplier_FR));
  analogWrite(BWDpin_FR, (0 * multiplier_FR));

  analogWrite(FWDpin_BL, (PWM * multiplier_BL));
  analogWrite(BWDpin_BL, (0 * multiplier_BL));

  analogWrite(FWDpin_BR, (PWM * multiplier_BR));
  analogWrite(BWDpin_BR, (0 * multiplier_BR));
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