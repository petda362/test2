// Projectgroup 1
// Bachelors-project in Electronics Design engineering

// Alexander Riex ED4
// Henrik Nilsson ED3
// Konrad Råström ED3
// Petter Danev ED5

//-------------Libraries---------------
#include <Arduino.h>
#include <math.h>

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

//--------Changeable variables-----------
int tolerance = 10;                 // Allowed error before the translation corection algorithm is implemented
int tolerance_angle = 25;           // Allowed error before the rotational correction algoritm is implemented
float angle = 18.33;                // Angle of the sensors from the vehicle
int PWM = 100;                      // Base PWM before modifiers. from 0 to 255
float multiplier_FL = 0.90;         // Multipliers for seperate wheels, for adjusting motor speed (PWM * multiplier)
float multiplier_FR = 0.90;
float multiplier_BL = 1.0;
float multiplier_BR = 0.90;
float multiplier_rotation = 0.8;    // multiplier for rotation, for adjusting motor speed whilst rotating (PWM * multiplier)

// --------Defining variables------------
long travelTime_FL;
long travelTime_FR;
long travelTime_BL;
long travelTime_BR;

int distance_FL;
int distance_FR;
int distance_BL;
int distance_BR;

double real_distance_FL;
double real_distance_FR;
double real_distance_BL;
double real_distance_BR;

bool orth = false;          // bolean that describes if the vehicle is parallell to the walls

//--------------Setup-------------------
void setup()
{
  pinMode(trigpin_FL, OUTPUT);
  pinMode(echopin_FL, INPUT);
  pinMode(trigpin_FR, OUTPUT);
  pinMode(echopin_FR, INPUT);
  pinMode(trigpin_BL, OUTPUT);
  pinMode(echopin_BL, INPUT);
  pinMode(trigpin_BR, OUTPUT);
  pinMode(echopin_BR, INPUT);

  pinMode(FWDpin_FL, OUTPUT);
  pinMode(BWDpin_FL, OUTPUT);
  pinMode(FWDpin_FR, OUTPUT);
  pinMode(BWDpin_FR, OUTPUT);
  pinMode(FWDpin_BL, OUTPUT);
  pinMode(BWDpin_BL, OUTPUT);
  pinMode(FWDpin_BR, OUTPUT);
  pinMode(BWDpin_BR, OUTPUT);

  Serial.begin(9600);                                            // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("Ultrasonic Sensor HC-SR04 Test, translation"); // print some text in Serial Monitor
}



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


//-----------Main loop-------------------------------------
void loop()
{
  // Calculates the orthogonal distance from the wall to the sensor based on the snesor angle
  real_distance_FL = real_distance(ultraSensor(trigpin_FL, echopin_FL), angle);
  real_distance_FR = real_distance(ultraSensor(trigpin_FR, echopin_FR), angle);
  real_distance_BL = real_distance(ultraSensor(trigpin_BL, echopin_BL), angle);
  real_distance_BR = real_distance(ultraSensor(trigpin_BR, echopin_BR), angle);


  // Rotational correction. ensures vehicle is parallel to the walls
  if ((real_distance_BL < (real_distance_FL - tolerance_angle)) ||
           (real_distance_FR < (real_distance_BR - tolerance_angle)))
  {
    rotate_centered_cclkw();
    orth = false;
  }
  else if ((real_distance_FL < (real_distance_BL - tolerance_angle) ||
            (real_distance_FR > (real_distance_BR + tolerance_angle))))
  {
    rotate_centered_clkw();
    orth = false;
  }
  else if (!orth) {
    orth = true;
    rotate_stop();
  }


  // Trnslational correction. Vehicle will attempt to center itself between two obstacles, after it has been rotated to parallel
  // only runs if orthogonal = True
  if (orth && ((real_distance_FL >= (real_distance_FR - tolerance)) &&
               (real_distance_FL <= (real_distance_FR + tolerance))))
  {
    translate_stop();
  }
  else if (orth && ((real_distance_FL < (real_distance_FR - tolerance)) || 
                    (real_distance_BL < (real_distance_BR - tolerance))))
  {
    translate_right();
  }
  else if (orth && ((real_distance_FL > (real_distance_FR + tolerance)) || 
                    (real_distance_BL > (real_distance_BR + tolerance))))
  {
    translate_left();
  }
}


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
  Serial.println("move right");

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

  Serial.println("move left");

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
