// Projectgroup 1
// Bachelors-project in Electronics Design Engineering
// 2022

// Alexander Riex ED4
// Henrik Nilsson ED3
// Konrad Råström ED3
// Petter Danev ED5

//-------------Libraries---------------
#include <Arduino.h>
#include "SoftwareSerial.h"
#include <math.h>
#include "correction.h"
#include "bluetooth.h"
#include "movement.h"
#include "audio.h"
#include "Servo.h"

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

// Servo ----------------
#define servo_pin 2
Servo plockservo;


//--------Changeable variables-----------
int tolerance = 2;                 // Allowed error before the translation correction algorithm is implemented
int tolerance_angle = 2;           // Allowed error before the rotational correction algoritm is implemented
float angle = 18.33;                // Angle of the sensors from the vehicle
int start_pwm = 200;                      // Base PWM before modifiers. from 0 to 255
int correction_pwm = 60;
int startup_sound = 0;              // if 1 = sing if 0 dont sing
bool plock = false;
int PWM_3 = 70; // PWM för zon 3. Måste kalibreras!
bool corrected = false;


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

double last_real_distance_FL;
double last_real_distance_FR;
double last_real_distance_BL;
double last_real_distance_BR;

int sensorValue1, sensorValue2, sensorValue3, sensorValue4, sensorValue5, sensorValue6, sensorValue7, sensorValue8;
int sensorValue9, sensorValue10, sensorValue11, sensorValue12, sensorValue13, sensorValue14, sensorValue15, sensorValue16;


double rotate_delay;
double z;

bool nope = false;
bool turned = false;
bool goagain = false;
bool biggus = false;
bool cp_variabel = false;

String BTBYTE;  // Received signal string.

String Empty;
char tempCommand[4];// variable for new string
String Command;

SoftwareSerial BTSerial(13,12); // RX , TX

char State1;
char Zone;
char Instruction;
char Ack;
char direction;
//int i = 0;
bool ignore = true;
int Tape = 0;
bool L_lost;
bool R_lost;
bool read_error;
char last_inst;
char last_Zone;

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
  
  sing(startup_sound);

}

// ----------Functions------------------------------------

void readIRData()
{
   sensorValue1 = analogRead(SENSORA_F1);
   sensorValue2 = analogRead(SENSORA_F2);
   sensorValue3 = analogRead(SENSORA_F3);
   sensorValue4 = analogRead(SENSORA_F4);
   sensorValue5 =  analogRead(SENSORA_F5);
   sensorValue6 = analogRead(SENSORA_F6);
   sensorValue7 = analogRead(SENSORA_F7);
   sensorValue8 =  analogRead(SENSORA_F8);

   sensorValue9 = analogRead(SENSORA_B1);
   sensorValue10 = analogRead(SENSORA_B2);
   sensorValue11 = analogRead(SENSORA_B3);
   sensorValue12 = analogRead(SENSORA_B4);
   sensorValue13 =  analogRead(SENSORA_B5);
   sensorValue14 = analogRead(SENSORA_B6);
   sensorValue15 = analogRead(SENSORA_B7);
   sensorValue16 =  analogRead(SENSORA_B8);

}

void Plockat() // funktion för att plocka klossen från hyllan
{
    for(int pos=0; pos <= 120; pos+=10){
        plockservo.write(pos);
        delay(15);
    }
delay(300);
plockservo.write(0);
}

String Tapestop(int nr, int PWM,String outmes) // funktion för att stanna vid tejp nr
{
Tape = 0;
        translate_FWD(PWM);
        while(Tape < nr){
            readIRData();
            if (((sensorValue9+sensorValue10+sensorValue15+sensorValue16+400) / 4) < ((sensorValue1+sensorValue2+sensorValue7+sensorValue8) / 4)){
                Tape = Tape + 1;
                if (Tape == nr){
                    delay(220);
                    quickbrake(PWM);
                    outmes[2] = 'p';
                }
                else{
                  BTSerial.println(Ack+"0p");
                }
                delay(100);
            }
     
        }
        translate_stop();
        return outmes;
}

String Instructions(char inst, int PWM, String Outmes_inst, char Zone)
{

    switch (inst)
    {
    case 's':
        translate_stop();
        rotate_stop();
        last_inst = 's';
        Outmes_inst[1]='1';  //Klar
        break;
    
    case 'f':
    translate_FWD(PWM);
    delay(800);
    if (last_Zone == 2){
        delay(300); 
    }

        //ignore = true;
        L_lost = false;
        R_lost = false;
        while (true){
            
        last_real_distance_FL = real_distance_FL;
        last_real_distance_FR = real_distance_FR;
        last_real_distance_BL = real_distance_BL;
        last_real_distance_BR = real_distance_BR;
  
        readIRData();
            if (((sensorValue9+sensorValue10+sensorValue15+sensorValue16+400) / 4) < ((sensorValue1+sensorValue2+sensorValue7+sensorValue8) / 4)){
                
                  BTSerial.println(Ack+"0p");

                delay(30);
            }

        real_distance_FL = real_distance(ultraSensor(trigpin_FL, echopin_FL), angle);
        real_distance_FR = real_distance(ultraSensor(trigpin_FR, echopin_FR), angle);
        real_distance_BL = real_distance(ultraSensor(trigpin_BL, echopin_BL), angle);
        real_distance_BR = real_distance(ultraSensor(trigpin_BR, echopin_BR), angle);
        
        
       /* while((real_distance_FL+real_distance_BL - real_distance_BR+real_distance_FR) < 10){
        rotational_correction(real_distance_FL,real_distance_BL, real_distance_FR, real_distance_BR,tolerance_angle,PWM/3);
        translational_correction(real_distance_FL,real_distance_BL, real_distance_FR, real_distance_BR,tolerance,PWM/3);
        }*/
       
        if(real_distance_FL > (last_real_distance_FL + 50)){
            L_lost = true;
        }
        if(real_distance_FR > (last_real_distance_FR + 50)){
            R_lost = true;
        }

        if (Zone == '4'){
            if(L_lost == true || R_lost == true){
                delay(250);
                break;
            }
        }
        else if(R_lost && L_lost){
            delay(250);
            break;
        }
        //ignore = false;
        if(Zone != '2' && L_lost == false && R_lost == false){
            if(abs(real_distance_FL-real_distance_FR) > 25 || abs(real_distance_FL-real_distance_FR) > 25 ){
                quickbrake(PWM);
                delay(50);
                total_correction(tolerance_angle,tolerance, PWM_3-3, angle);
                delay(50);
                translate_FWD(PWM);
                delay(10);
        }
        }
        }
        translate_stop();
        Outmes_inst[1]='1';  //Klar
        last_inst = 'f';
        break;
    
    case 'b':
        translate_BWD(PWM);
        delay(250);
        Outmes_inst[1]='1';  //Klar
        last_inst = 'b';
        break;

    case 'l':
    break;

    case 'm': 
    Plockat();
    sing(1);
    Outmes_inst[2]='u';  // Plockning utförd
    Outmes_inst[1]='1';  // Klar
    last_inst = 'm';
    break;

    case 'h':
        rotate_centered_clkw(PWM);
        delay(610);
        rotate_stop();
        Outmes_inst[1]='1';
        last_inst = 'h';       
        break;
    
    case 'L':
    sing(3);
    Outmes_inst[1] = '1';
    last_inst = 'L';
    break;

    case 'v':
        rotate_centered_cclkw(PWM);
        delay(610);
        rotate_stop();
        Outmes_inst[1]='1';  
        last_inst = 'v';     
        break;

    case 'x': 
         Outmes_inst = Tapestop(1,PWM,Outmes_inst);
        Outmes_inst[1]='1';  //Klar
        last_inst = 'x';
        break;

    case 'y': 
        Outmes_inst = Tapestop(2,PWM,Outmes_inst);
        Outmes_inst[1]='1';  //Klar
        last_inst = 'y';
        break;

    case 'z': 
        Outmes_inst = Tapestop(3,PWM,Outmes_inst);
        Outmes_inst[1]='1';  //Klar
        last_inst = 'z';
        break;

    case 'C':
        total_correction(tolerance_angle, tolerance, PWM_3, angle);
        Outmes_inst[1] = '1'; //Klar
        last_inst = 'C';
        break;
        

    default:
        // Ogiltig instruktions karaktär.
        Outmes_inst[2] = 'f';
        break;
    }
    last_Zone = Zone;
    return Outmes_inst;
}

String readBluetoothData(String BTBYTE, int PWM, bool plock)    // PWM för zon 1 och 2.
{
    plockservo.attach(servo_pin);

    if (plock == false){
        Plockat();   
        plock = true;
    }
    
    String Out_mes="000";  // Return string.
    // Dela upp BTBYTE meddelanden (*/*/*/*).
    Out_mes[0]=BTBYTE[0];    // Ack.
    State1=BTBYTE[1];       // Auto eller manuell.
    Zone=BTBYTE[2];         // Vilken zon.
    Instruction=BTBYTE[3];  // Instruktion.
  

    //direction=BTBYTE[0];
    //Serial.println("TEST");
    
    switch (State1) // Ta bort???
    {
    case '0':   // Manuell
    case '1':   // Automatisk
            switch (Zone)
            {
                case '1':   // Zone 1
                        Out_mes=Instructions(Instruction, PWM, Out_mes, Zone);
                        //return "In zone 1";
                    break;
    
                case '2':   // Zone 2
                        Out_mes=Instructions(Instruction, PWM, Out_mes, Zone);
                        // if IR-sensor har hittat linje return INBYTE[2]=p
                        //return "In zone 2";
                    break;
        
                case '3':   // Zone 3
                        Out_mes=Instructions(Instruction, PWM_3, Out_mes,Zone);
                        //return "In zone 3";
                    break;
                
                case '4': //Zone 4 (hylla på ena sidan sarg på andra)
                         Out_mes=Instructions(Instruction, PWM, Out_mes,Zone);
                         break;

                default:    // Ingen zone.
                    Out_mes[2]='f';    //
                    break;
            }
        break;
        
    default:
        break;
    }
    return Out_mes;
}



//-----------Main loop-------------------------------------
void loop()
{
String INBYTE;  // Transmitting signal string.
if(BTSerial.available())    // Till AGV    
{
    BTBYTE=BTSerial.readString();
    //bygger ny string av BTbyte som inte har det osynliga tecknet
    Command ="";
    read_error = false;
    for (int i = 0; i < 4; ++i){
        if(BTBYTE[i]=='\0'){
            read_error = true;
            break;
        }
        else{
            Command = Command + BTBYTE[i];
        }
    }
    if (read_error == false){
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
    else{
        //BTSerial.println("Error reading input");
    }
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
/*
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
*/
}


