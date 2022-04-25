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

#define max_distance 500 // maximum ultrasonic sensor distance

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


#define SENSOR_L 23
#define SENSOR_R 22


unsigned long time_echo_FL, time_echo_FR, time_echo_BL, time_echo_BR;
double last_real_distance_FL;
double last_real_distance_FR; 
double last_real_distance_BL;
double last_real_distance_BR;

double real_distance_FL;
double real_distance_FR;
double real_distance_BL;
double real_distance_BR;



  long travelTime;
  float distance;
  float sum_distances = 0;
  float avg_distance;
  int num_measurements = 5;

//////////////////////////PID///////////////////////////////////
unsigned long time_last, time_current;
float time_diff;
float error, last_error, P, I, D, pid_output;
float rot_error_front, rot_error_back, P_rot_front, P_rot_back, I_rot_front, D_rot_front, I_rot_back, D_rot_back, last_rot_error_front, last_rot_error_back;

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
  sum_distances = 0;
  for (int i = 0; i < num_measurements; i++) {
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
    rotate_stop();
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

  /*
  real_distance_FL = real_distance(ultraSensor(trigpin_FL, echopin_FL), 18.33);
  real_distance_FR = real_distance(ultraSensor(trigpin_FR, echopin_FR), 18.33);
  real_distance_BL = real_distance(ultraSensor(trigpin_BL, echopin_BL), 18.33);
  real_distance_BR = real_distance(ultraSensor(trigpin_BR, echopin_BR), 18.33);
  */

/*
    for(int i = 0; i<5; i++)
    {
    real_distance_FL += (((sonarFL.ping()) / (US_ROUNDTRIP_CM))) * sin(DEG_TO_RAD * 18.33);
    real_distance_FR += (((sonarFR.ping()) / (US_ROUNDTRIP_CM))) * sin(DEG_TO_RAD * 18.33);
    real_distance_BL += (((sonarBL.ping()) / (US_ROUNDTRIP_CM))) * sin(DEG_TO_RAD * 18.33);
    real_distance_BR += (((sonarBR.ping()) / (US_ROUNDTRIP_CM))) * sin(DEG_TO_RAD * 18.33);
    delay(30);
    }
    real_distance_FL = (real_distance_FL / 5);
    real_distance_FR = (real_distance_FR / 5);
    real_distance_BL = (real_distance_BL / 5);
    real_distance_BR = (real_distance_BR / 5);

*/

time_echo_FL = 0;
time_echo_FR = 0;
time_echo_BL = 0;
time_echo_BR = 0;

for(int i = 0; i<3; i++)
{
  time_echo_FL += sonarFL.ping();
  time_echo_FR += sonarFR.ping(); 
  time_echo_BL += sonarBL.ping(); 
  time_echo_BR += sonarBR.ping();
}

time_echo_FL = time_echo_FL / 3;
time_echo_FR = time_echo_FR / 3;
time_echo_BL = time_echo_BL / 3;
time_echo_BR = time_echo_BR / 3;

real_distance_FL = sonarFL.convert_cm(time_echo_FL);
real_distance_FR = sonarFR.convert_cm(time_echo_FR);
real_distance_BL = sonarBL.convert_cm(time_echo_BL);
real_distance_BR = sonarBR.convert_cm(time_echo_BR);

delay(25);

/*
real_distance_FL = real_distance_FL * sin(DEG_TO_RAD * 18.33);
real_distance_FR = real_distance_FR * sin(DEG_TO_RAD * 18.33); 
real_distance_BL = real_distance_BL * sin(DEG_TO_RAD * 18.33); 
real_distance_BR = real_distance_BR * sin(DEG_TO_RAD * 18.33); 
*/




  
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

/*
    rot_error_front = real_distance_FL - real_distance_FR;
    rot_error_back = real_distance_BL - real_distance_BR;
    
    P_rot_front = rot_error_front;
    P_rot_back = rot_error_back;
    I_rot_front = rot_error_front * time_diff;
    I_rot_back = rot_error_back * time_diff;
    D_rot_front = (rot_error_front - last_rot_error_front)/time_diff;
    D_rot_back = (rot_error_back - last_rot_error_back)/time_diff;*/
    
    error = (real_distance_FL - real_distance_BL) + (real_distance_BR - real_distance_FR);

    P = error;
    I += error*time_diff;
    D = (error - last_error)/time_diff;

    pid_output = Kp*P + Ki*I + Kd*D;
    

    if(pid_output > pid_max){pid_output = pid_max;}
    if(pid_output < pid_min){pid_output = pid_min;}
    
    //Serial.print("pid: ");
    //Serial.println(pid_output);
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

    if(((real_distance_FL == 0 && last_real_distance_FL < 300 && last_real_distance_FL > 20) || (real_distance_FR == 0 && last_real_distance_FR < 300 && last_real_distance_FR > 20)) || ((real_distance_FL - last_real_distance_FL > 200) || (real_distance_FL - last_real_distance_FL > 200))){
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
    if(real_distance_FL - real_distance_BL > 15 || real_distance_BR - real_distance_FR > 15)
    {
        rotate_centered_cclkw(80);
    }
    else if(real_distance_BL - real_distance_FL > 15 || real_distance_FR - real_distance_BR > 15)
    {
        rotate_centered_clkw(80);
    }
    else
    {
      translate_stop();
    }
  }
}

void pickLeftCube()
{
    int leftSensorHit = 0;
   // bool testLeft = false;
   // bool testRight = false;
   // int rightSensorHit = 0;
    translate_left(80);
    while(true)
    {
    leftSensorHit = digitalRead(SENSOR_L);
   // rightSensorHit = digitalRead(SENSOR_R);
    if(leftSensorHit == 0)
    {
      //testLeft = true;
      while(leftSensorHit==0){
        leftSensorHit = digitalRead(SENSOR_L);
      //do nothing
      }
      quickbrake(80);
      break;
    
    }
    
    }
}

