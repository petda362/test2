// Projectgroup 1
// Bachelors-project in Electronics Design engineering

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

// --------Defining variables------------
long travelTime_FL;
long travelTime_FR;
long travelTime_BL;
long travelTime_BR;

int distance_FL;
int distance_FR;
int distance_BL;
int distance_BR;

String BTBYTE;  // Received signal string.
String INBYTE;  // Transmitting signal string.

double real_distance_FL;
double real_distance_FR;
double real_distance_BL;
double real_distance_BR;

String OS_signal; // ÖS_Signal

SoftwareSerial BTSerial(13,12); // RX , TX


// -------------------------Setup-------------------
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

  Serial.begin(9600);                                            // Serial Communication is starting with 9600 of baudrate speed
  BTSerial.begin(9600);
  Serial.println("Ultrasonic Sensor HC-SR04 Test, translation."); // print some text in Serial Monitor
  BTSerial.println("Connected to AGV.");  // Print on bluetooth device.

}

//-----------Main loop-------------------------------------
void loop()
{
  // Calculates the orthogonal distance from the wall to the sensor based on the sensor angle
  //real_distance_FL = real_distance(ultraSensor(trigpin_FL, echopin_FL), angle);
  //real_distance_FR = real_distance(ultraSensor(trigpin_FR, echopin_FR), angle);
  //real_distance_BL = real_distance(ultraSensor(trigpin_BL, echopin_BL), angle);
  //real_distance_BR = real_distance(ultraSensor(trigpin_BR, echopin_BR), angle);

  //rotational_correction(real_distance_FL, real_distance_BL, real_distance_FR, real_distance_BR, tolerance_angle);

  //translational_correction(real_distance_FL, real_distance_BL, real_distance_FR, real_distance_BR, tolerance);

  //OS_signal = readBluetoothData();
  

int l = 0;      // l = length of recieved signal.
if(BTSerial.available())    // Till AGV    
{
    BTBYTE=BTSerial.readString();
    INBYTE=readBluetoothData(BTBYTE); // Behandla meddelandet. Returnera meddelande som ska tillbaka till ÖS.
    //l=BTBYTE.length();              // Längden på BTBYTE.
    //Serial.println("111");          // Test
    //BTSerial.println(BTBYTE);
    //Serial.print(BTBYTE);           // Skriver ut BTBYTE i Serial (Serialen på arduino:n).
    Serial.print(BTBYTE);
    //Serial.print(BTBYTE.substring(1, 8)); // substring(startpunkt, slutpunkt+1)
    //Serial.print(", ");
    BTSerial.println(INBYTE);     // Send string message to serial.
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

    //INBYTE = Serial.readString();      
    //BTSerial.println(INBYTE);       // Skriver ut INBYTE i BTSerial (Bluetooth Transmitterns Serialen).
    //Serial.print(INBYTE);
    //Serial.println("222");      // Test
}

}