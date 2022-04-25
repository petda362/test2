// Projectgroup 1
// Bachelors-project in Electronics Design engineering
// 2022

// Alexander Riex ED4
// Henrik Nilsson ED3
// Konrad Råström ED3
// Petter Danev ED5

//-------------Libraries---------------
#include "correction.h"
#include "movement.h"
#include <NewPing.h>

//---------Defining pins----------------

#define echopin_FL 40 // Forward Left sensor
#define trigpin_FL 41
#define echopin_FR 42 // Forward Right sensor
#define trigpin_FR 43
#define echopin_BL 44 // Back Left
#define trigpin_BL 45
#define echopin_BR 46 // Back Right
#define trigpin_BR 47

#define max_distance 350 // maximum ultrasonic sensor distance

NewPing sonarFL(trigpin_FL, echopin_FL, max_distance);
NewPing sonarFR(trigpin_FR, echopin_FR, max_distance);
NewPing sonarBL(trigpin_BL, echopin_BL, max_distance);
NewPing sonarBR(trigpin_BR, echopin_BR, max_distance);


#define STATE 50      // STATE PIN HC05

#define FWDpin_FL 9 // FWD Forward left
#define BWDpin_FL 8 // BWD
#define FWDpin_FR 4 // FWD Forward Right
#define BWDpin_FR 5 // BWD
#define FWDpin_BL 7 // FWD Backward LefB
#define BWDpin_BL 6 // BWD
#define FWDpin_BR 10 // FWD Backward Right
#define BWDpin_BR 11 // BWD


double last_real_distance_FL;
double last_real_distance_FR; 
double last_real_distance_BL;
double last_real_distance_BR;
double real_distance_FL;
double real_distance_FR;
double real_distance_BL;
double real_distance_BR;


//////////////////////////PID///////////////////////////////////
unsigned long time_last, time_current;
float time_diff;
float error, last_error, P, I, D, pid_output;


float Kp = 0.6;
float Ki = 0;
float Kd = 0;

const float pid_max = 50;
const float pid_min = -50;
///////////////////////////////////////////////////////////////




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
  float sum_distances = 0;
  float avg_distance;
  int num_measurements = 5;
  
  for (int i = 0; i <= num_measurements; i = i + 1) {
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

void rotational_correction (double dist_FL, double dist_BL, double dist_FR, double dist_BR, int tolerance_angle, int PWM) {
    if ((dist_BL < (dist_FL - tolerance_angle)) ||
           (dist_FR < (dist_BR - tolerance_angle)))
  {
    rotate_centered_cclkw(PWM);
    orth = false;
  }
  else if ((dist_FL < (dist_BL - tolerance_angle) ||
            (dist_FR > (dist_BR + tolerance_angle))))
  {
    rotate_centered_clkw(PWM);
    orth = false;
  }
  else if (!orth) {
    orth = true;
    quickbrake(PWM);
    // rotate_stop();
  }
}

void translational_correction (double dist_FL, double dist_BL, double dist_FR, double dist_BR, int tolerance, int PWM) {

    if (orth && ((dist_FL >= (dist_FR - tolerance)) &&
               (dist_FL <= (dist_FR + tolerance))))
  {
    translate_stop();
  
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

void readUltraSensors()
{
  last_real_distance_FL = real_distance_FL;
  last_real_distance_FR = real_distance_FR;
  last_real_distance_BL = real_distance_BL;
  last_real_distance_BR = real_distance_BR;

  for(int i = 0; i<3; i++)
  {
    real_distance_FL += (((sonarFL.ping()) / (US_ROUNDTRIP_CM)) * 10) * sin(DEG_TO_RAD * 18.33);
    real_distance_FR += (((sonarFR.ping()) / (US_ROUNDTRIP_CM)) * 10) * sin(DEG_TO_RAD * 18.33);
    real_distance_BL += (((sonarBL.ping()) / (US_ROUNDTRIP_CM)) * 10) * sin(DEG_TO_RAD * 18.33);
    real_distance_BR += (((sonarBR.ping()) / (US_ROUNDTRIP_CM)) * 10) * sin(DEG_TO_RAD * 18.33);
    delay(30);
  }
  real_distance_FL = real_distance_FL / 3;
  real_distance_FR = real_distance_FR / 3;
  real_distance_BL = real_distance_BL / 3;
  real_distance_BR = real_distance_BR / 3;

  

  
  Serial.print("FL: ");
  Serial.print(real_distance_FL);
  Serial.print("\t");
  Serial.print("FR: ");
  Serial.print(real_distance_FR);
  Serial.print("\t");
  Serial.print("BL: ");
  Serial.print(real_distance_BL);
  Serial.print("\t");
  Serial.print("BR: ");
  Serial.println(real_distance_BR);
  

}

void calcPID()
{
    time_last = time_current;
    time_current = millis();
    time_diff = (float)(time_current - time_last)/1000;

    error = real_distance_FL - real_distance_FR;
    
    P = error; 
    I += error*time_diff;
    D = (error - last_error)/time_diff;
    
    pid_output = Kp*P + Ki*I + Kd*D;
    

    if(pid_output > pid_max){pid_output = pid_max;}
    if(pid_output < pid_min){pid_output = pid_min;}
    
    Serial.print("pid: ");
    Serial.println(pid_output);
}


void stopAtEdge()
{
  bool first_it = true;
  while (true){
    translate_FWD(200);
    if(first_it)
    {
      delay(750);
      first_it = false;
    }
    readUltraSensors();
    if(((last_real_distance_FL < 100 && last_real_distance_FL > 20) || (last_real_distance_FR < 100 && last_real_distance_FR > 20)) && ((real_distance_FL - last_real_distance_FL > 100) || (real_distance_FR - last_real_distance_FR > 100) || (real_distance_FL == 0) || (real_distance_FR == 0))){
            delay(180);
            translate_stop();
            break;
    }
    delay(10);
  }
}

void stopAtShelf()
{
    translate_FWD(60);
    delay(1660);
    translate_stop();
}

void centerBetweenShelfs()
{
  while(true)
  {
    readUltraSensors();
    rotational_correction(real_distance_FL,real_distance_BL,real_distance_FR,real_distance_BR,16,70);
    translational_correction(real_distance_FL,real_distance_BL,real_distance_FR,real_distance_BR,6,70);
  }
}

