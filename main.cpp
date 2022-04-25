// Projectgroup 1
// Bachelors-project in Electronics Design Engineering
// 2022

// Alexander Riex ED4
// Henrik Nilsson ED3
// Konrad Råström ED3
// Petter Danev ED5
// Ta bort

//-------------Libraries---------------
#include <Arduino.h>
#include <math.h>
#include "correction.h"
#include "bluetooth.h"
#include "SoftwareSerial.h"
#include "movement.h"
#include "audio.h"

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


#define SENSORA_F1 A0   // Sensor ramp forward
#define SENSORA_F2 A1
#define SENSORA_F3 A2
#define SENSORA_F4 A3
#define SENSORA_F5 A4
#define SENSORA_F6 A5
#define SENSORA_F7 A6
#define SENSORA_F8 A7

#define SENSORA_B1 A8   // sensor ramp backward
#define SENSORA_B2 A9
#define SENSORA_B3 A10
#define SENSORA_B4 A11
#define SENSORA_B5 A12
#define SENSORA_B6 A13
#define SENSORA_B7 A14
#define SENSORA_B8 A15

//--------Changeable variables-----------
//int tolerance = 3;                 // Allowed error before the translation corection algorithm is implemented
//int tolerance_angle = 20;           // Allowed error before the rotational correction algoritm is implemented
//float angle = 18.33;                // Angle of the sensors from the vehicle
int start_pwm = 200;                      // Base PWM before modifiers. from 0 to 255
int correction_pwm = 60;
int startup_sound = 1;              // if 1 = sing if 0 dont sing
bool plock = false;

// --------Defining variables------------
long travelTime_FL;
long travelTime_FR;
long travelTime_BL;
long travelTime_BR;

int distance_FL;
int distance_FR;
int distance_BL;
int distance_BR;

//double real_distance_FL;
//double real_distance_FR;
//double real_distance_BL;
//double real_distance_BR;

//double last_real_distance_FL;
//double last_real_distance_FR;
//double last_real_distance_BL;
//double last_real_distance_BR;

double rotate_delay;
double z;

String BTBYTE;  // Received signal string.
String INBYTE;  // Transmitting signal string.
String Empty;
char tempCommand[4];// variable for new string
String Command;

SoftwareSerial BTSerial(13,12); // RX , TX

// -------------------------Setup-------------------
void setup()
{
  /*
  pinMode(trigpin_FL, OUTPUT);
  pinMode(echopin_FL, INPUT);
  pinMode(trigpin_FR, OUTPUT);
  pinMode(echopin_FR, INPUT);
  pinMode(trigpin_BL, OUTPUT);
  pinMode(echopin_BL, INPUT);
  pinMode(trigpin_BR, OUTPUT);
  pinMode(echopin_BR, INPUT);
  */

  pinMode(FWDpin_FL, OUTPUT);
  pinMode(BWDpin_FL, OUTPUT);
  pinMode(FWDpin_FR, OUTPUT);
  pinMode(BWDpin_FR, OUTPUT);
  pinMode(FWDpin_BL, OUTPUT);
  pinMode(BWDpin_BL, OUTPUT);
  pinMode(FWDpin_BR, OUTPUT);
  pinMode(BWDpin_BR, OUTPUT);

  Serial.begin(9600);                                            // Serial Communication is starting with 9600 of baudrate speed
  BTSerial.begin(9600);
  
  // IR sensorernas input.
  pinMode(SENSORA_F1,INPUT);
  pinMode(SENSORA_F2,INPUT);
  pinMode(SENSORA_F3,INPUT);
  pinMode(SENSORA_F4,INPUT);
  pinMode(SENSORA_F5,INPUT);
  pinMode(SENSORA_F6,INPUT);
  pinMode(SENSORA_F7,INPUT);
  pinMode(SENSORA_F8,INPUT);

  pinMode(SENSORA_B1,INPUT);
  pinMode(SENSORA_B2,INPUT);
  pinMode(SENSORA_B3,INPUT);
  pinMode(SENSORA_B4,INPUT);
  pinMode(SENSORA_B5,INPUT);
  pinMode(SENSORA_B6,INPUT);
  pinMode(SENSORA_B7,INPUT);
  pinMode(SENSORA_B8,INPUT);

  pinMode(buzzer_pin, OUTPUT);
  
  //sing(startup_sound);

}

//-----------Main loop-------------------------------------
void loop()
{
  // Calculates the orthogonal distance from the wall to the sensor based on the sensor angle
//   real_distance_FL = real_distance(ultraSensor(trigpin_FL, echopin_FL), angle);
//   real_distance_FR = real_distance(ultraSensor(trigpin_FR, echopin_FR), angle);
//   real_distance_BL = real_distance(ultraSensor(trigpin_BL, echopin_BL), angle);
//   real_distance_BR = real_distance(ultraSensor(trigpin_BR, echopin_BR), angle);

//   rotational_correction(real_distance_FL, real_distance_BL, real_distance_FR, real_distance_BR, tolerance_angle, start_pwm);

//   translational_correction(real_distance_FL, real_distance_BL, real_distance_FR, real_distance_BR, tolerance, start_pwm);

//   last_real_distance_FL = real_distance_FL;
//   last_real_distance_FR = real_distance_FR;
//   last_real_distance_BL = real_distance_BL;
//   last_real_distance_BR = real_distance_BR;
  
//   real_distance_FL = real_distance(ultraSensor(trigpin_FL, echopin_FL), angle);
//   real_distance_FR = real_distance(ultraSensor(trigpin_FR, echopin_FR), angle);
//   real_distance_BL = real_distance(ultraSensor(trigpin_BL, echopin_BL), angle);
//   real_distance_BR = real_distance(ultraSensor(trigpin_BR, echopin_BR), angle);
  
//     // rotate_delay = -0.0004*pow(start_pwm, 3) + 0.2537*pow(start_pwm,2) - 53.176*(start_pwm) + 4288.3;
//     z = (start_pwm - 175) / 47.6;
//     rotate_delay = -45.1 * pow(z,3) + 77.5 * pow(z,2) - 133 * pow(z,1) + 510;
//readUltraSensors();
//calcPID();
//translate_FWD(200);

if(BTSerial.available())    // Till AGV    
{
    BTBYTE=BTSerial.readString();
    //bygger ny string av BTbyte som inte har det osynliga tecknet
    Command ="";
    for (int i = 0; i < 4; ++i){
    Command = Command + BTBYTE[i];
    }
    Serial.println(Command);
    //l=BTBYTE.length();              // Längden på BTBYTE.
    //Serial.println("111");          // Test
    //BTSerial.println(BTBYTE);
    //Serial.print(BTBYTE);           // Skriver ut BTBYTE i Serial (Serialen på arduino:n).
    //Serial.println(BTBYTE);
    INBYTE=readBluetoothData(Command, start_pwm, plock); // Behandla meddelandet. Returnera meddelande som ska tillbaka till ÖS.
    plock = true;
    //Serial.println("DÄR!!!");
    BTSerial.println(INBYTE);     // Send string message to serial.
    
}
if (Command == "ssss"){
  Serial.println("I stop funktion");
  delay(4000);
}
else{
  Serial.println(BTBYTE.length());
  Serial.println(BTBYTE);
 // Serial.println("HÄR!!!");
  delay(1000); //1 sek = 1'000ms.
}

if(Serial.available())              // Från AGV
{   
    INBYTE = Serial.readString();
    //INBYTE=readBluetoothData(BTBYTE);
    //l=BTBYTE.length();              // Längden på BTBYTE.
    //Serial.println("111");          // Test
    //BTSerial.println(BTBYTE);
    //Serial.print(BTBYTE);           // Skriver ut BTBYTE i Serial (Serialen på arduino:n).
    BTSerial.println(INBYTE);     // Send string message to serial.
//     // Serial.println(rotate_delay);
//     // rotate_centered_clkw(start_pwm);
//     // delay(rotate_delay);
//     // rotate_stop();
//     // delay(10000);
    

}
}

