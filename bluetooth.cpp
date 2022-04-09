//---------------------------------Libraries--------------------------
#include <Arduino.h>
#include <math.h>
#include "bluetooth.h"
#include "movement.h"
#include "SoftwareSerial.h"
#include "correction.h"
#include <Servo.h>
#include "audio.h"

// --------Defining variables------------
char State1;
char Zone;
char Instruction;
char Ack;
char direction;
int i = 0;
bool ignore = true;
int Tape = 0;


// sensor variables 

float angle = 18.33;
int tolerance_angle = 20; //tolerance for correction functions
int tolerance = 3; //tolerance for correction functions
int pppppp = 1; // used to loop


int sensorValue1, sensorValue2, sensorValue3, sensorValue4, sensorValue5, sensorValue6, sensorValue7, sensorValue8;
int sensorValue9, sensorValue10, sensorValue11, sensorValue12, sensorValue13, sensorValue14, sensorValue15, sensorValue16;


// --------------------- changeable variables-----------------

int PWM_3 = 60; // PWM för zon 3. Måste kalibreras!




//---------Defining pins----------------

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

#define echopin_FL 40 // Forward Left sensor
#define trigpin_FL 41
#define echopin_FR 42 // Forward Right sensor
#define trigpin_FR 43
#define echopin_BL 44 // Back Left
#define trigpin_BL 45
#define echopin_BR 46 // Back Right
#define trigpin_BR 47



#define servo_pin 2

Servo myservo;

// ------------------------------- function ---------------------------

String readBluetoothData(String BTBYTE, int PWM, bool plock)    // PWM för zon 1 och 2.
{
    myservo.attach(servo_pin);

    if (plock == false){
        Plockat();   
        plock = true;
    }
    
    String INBYTE="000";  // Return string.
    // Dela upp BTBYTE meddelanden (*/*/*/*).
    INBYTE[0]=BTBYTE[0];    // Ack.
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
                        INBYTE=Instructions(Instruction, PWM, INBYTE);
                        //return "In zone 1";
                    break;
    
                case '2':   // Zone 2
                        INBYTE=Instructions(Instruction, PWM, INBYTE);
                        // if IR-sensor har hittat linje return INBYTE[2]=p
                        //return "In zone 2";
                    break;
        
                case '3':   // Zone 3
                        INBYTE=Instructions(Instruction, PWM_3, INBYTE);
                        //return "In zone 3";
                    break;

                default:    // Ingen zone.
                    INBYTE[2]='f';    //
                    break;
            }
        break;
        
    default:
        break;
    }
    return INBYTE;
}

String Instructions(char inst, int PWM, String INBYTE)
{

    switch (inst)
    {
    case 's':
        translate_stop();
        rotate_stop();
        INBYTE[1]='1';  //Klar
        break;
    
    case 'f':
        stopAtEdge();
        INBYTE[1]='1';  //Klar
    
        break;
    
    case 'b':
        translate_BWD(PWM);
        delay(250);
        INBYTE[1]='1';  //Klar
        break;

    case 'l':
    break;

    case 'm': 
    Plockat();
    sing(1);
    INBYTE[2]='u';  // Plockning utförd
    INBYTE[1]='1';  // Klar
    break;

    case 'h':
        rotate_centered_clkw(PWM);
        delay(630);
        rotate_stop();
        INBYTE[1]='1';       
        break;
    
    case 'v':
        rotate_centered_cclkw(PWM);
        delay(630);
        rotate_stop();
        INBYTE[1]='1';       
        break;

    case 'x': 
         Tapestop(1,PWM);
        INBYTE[1]='1';  //Klar
        break;

    case 'y': 
        Tapestop(2,PWM);
        INBYTE[1]='1';  //Klar
        break;

    case 'z': 
         Tapestop(3,PWM);
        INBYTE[1]='1';  //Klar
        break;
    case 'F':
    stopAtShelf();
    break;


    default:
        // Ogiltig instruktions karaktär.
        INBYTE[2] = 'f';
        break;
    }
    return INBYTE;
}

String Plocka(char inst, int PWM, String INBYTE)
{
    int time=1000;  //1000ms=1s

switch (inst)
{
case 'l':   // Vänster plock
    translate_left(PWM);
    delay(time);    
    translate_stop();
    Plockat();
    translate_right(PWM);
    delay(time);    
    translate_stop();
    INBYTE[2]='u';  // Plockning utförd
    INBYTE[1]='1';  //Klar
    break;

case 'r':   // Höger Plock
    translate_right(PWM);
    delay(time);    
    translate_stop();
    Plockat();
    translate_left(PWM);
    delay(time);    
    translate_stop();
    INBYTE[2]='u';  // Plockning utförd
    INBYTE[1]='1';  //Klar
    break;

case 'm':
    
    Plockat();
    sing(1);
    INBYTE[2]='u';  // Plockning utförd
    INBYTE[1]='1';  // Klar
    break;
case 'q':
    Plockat();
    break;
default:
    break;
}

return INBYTE;
}

String Tejpbitar(char inst, int PWM, String INBYTE)
{
    // Loopa hela funktionen tills man hittar rätt antal tejpbitar.
    while(INBYTE[1] == '0') //Loop while looking for the coorect tejpbit.
    {
    readIRData();
    translate_FWD(PWM);
    // IR-sensor kod.
    if(((sensorValue9+sensorValue10+sensorValue11+sensorValue12+sensorValue13+sensorValue14+sensorValue15+sensorValue16 + 600) / 8) < 
    ((sensorValue1+sensorValue2+sensorValue3+sensorValue4+sensorValue5+sensorValue6+sensorValue7+sensorValue8) / 8))
    {
        i++;
        switch (inst)
    {
    case 'x':
        if (i==1)
        {
            INBYTE[1]='1';  // Klar
            INBYTE[2]='p';  // Skickar tillbaka att den har passerat en tejpbit
            i=0;
             
        }
        break;
    case 'y':
        if (i==2)
        {
            INBYTE[1]='1';  // Klar
            INBYTE[2]='p';  // Skickar tillbaka att den har passerat en tejpbit
            i=0;
        }
        break;
    case 'z':
        if (i==3)
        {
            INBYTE[1]='1';  // Klar
            INBYTE[2]='p';  // Skickar tillbaka att den har passerat en tejpbit.    
            i=0;
        }
        break;
    
    default:
        INBYTE[2]='p';  // Skickar tillbaka att den har passerat en tejpbit.
      //  BTSerial.println(INBYTE);
        break;
    }
    
    }   
    }    
return INBYTE;
}

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
//int calc1 = (sensorValue1 + sensorValue2 + sensorValue7 + sensorValue8);
//int calc2 = (sensorValue9 + sensorValue10 + sensorValue15 + sensorValue16);
  // Serial.println(calc1 , "    ", calc2);
}

void Plockat() // funktion för att plocka klossen från hyllan
{
    for(int pos=0; pos <= 120; pos+=10){
        myservo.write(pos);
        delay(15);
    }
delay(300);
myservo.write(0);
}

void Tapestop(int nr, int PWM) // funktion för att stanna vid tejp nr
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
                }
                delay(100);
            }
     
        }
        translate_stop();
}


