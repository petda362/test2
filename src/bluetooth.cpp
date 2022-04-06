//---------------------------------Libraries--------------------------
#include <Arduino.h>
#include <math.h>
#include "bluetooth.h"
#include "movement.h"
#include "SoftwareSerial.h"

// --------Defining variables------------
char State1;
char Zone;
char Instruction;
char Ack;
char direction;
int i=0;

int sensorValue1, sensorValue2, sensorValue3, sensorValue4, sensorValue5, sensorValue6, sensorValue7, sensorValue8;
int sensorValue9, sensorValue10, sensorValue11, sensorValue12, sensorValue13, sensorValue14, sensorValue15, sensorValue16;
//int tapeCount = 0;

// --------------------- changeable variables-----------------

int PWM_3 = 60; // PWM för zon 3. Måste kalibreras!

//---------Defining pins----------------

#define SENSORA_F1 A4   // Sensor ramp forward
#define SENSORA_F2 A5
#define SENSORA_F3 A6
#define SENSORA_F4 A7
#define SENSORA_F5 A0
#define SENSORA_F6 A1
#define SENSORA_F7 A2
#define SENSORA_F8 A3

#define SENSORA_B1 A8   // sensor ramp backward
#define SENSORA_B2 A12
#define SENSORA_B3 A9
#define SENSORA_B4 A13
#define SENSORA_B5 A10
#define SENSORA_B6 A14
#define SENSORA_B7 A11
#define SENSORA_B8 A15

// ------------------------------- function ---------------------------

String readBluetoothData(String BTBYTE, int PWM)    // PWM för zon 1 och 2.
{
    String INBYTE="000";  // Return string.
    // Dela upp BTBYTE meddelanden (*/*/*/*).
    State1=BTBYTE[0];       // Auto eller manuell.
    Zone=BTBYTE[1];         // Vilken zon.
    Instruction=BTBYTE[2];  // Instruktion.
    INBYTE[0]=BTBYTE[3];    // Ack.
    //direction=BTBYTE[0];
    Serial.println("TEST");
    
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

String Instructions(char inst, int PWM, String INBYTE){

    switch (inst)
    {
    case 's':
        translate_stop();
        rotate_stop();
        INBYTE[1]='1';  //Klar
        break;
    
    case 'f':
        translate_FWD(PWM);
        delay(1000);
        translate_stop();
        INBYTE[1]='1';  //Klar
        break;
    
    case 'b':
        translate_BWD(PWM);
        INBYTE[1]='1';  //Klar
        break;

    case 'l':   
    case 'm':
    case 'h':
        INBYTE=Plocka(inst, PWM, INBYTE);        
        break;

    case 'x':
    case 'y':
    case 'z':
        INBYTE=Tejpbitar(inst, PWM, INBYTE);
        break;

    default:
        // Ogiltig instruktions karaktär.
        break;
    }
    return INBYTE;
}

String Plocka(char inst, int PWM, String INBYTE)
{
    int time=1000;  //1000ms=1s

switch (inst)
{
case 'l':   // Vänster block
    translate_left(PWM);
    delay(time);    
    translate_stop();
    // Plocka
    translate_right(PWM);
    delay(time);    
    translate_stop();
    INBYTE[2]='u';  // Plockning utförd
    INBYTE[1]='1';  //Klar
    break;

case 'h':   // Vänster block
    translate_right(PWM);
    delay(time);    
    translate_stop();
    // Plocka
    translate_left(PWM);
    delay(time);    
    translate_stop();
    INBYTE[2]='u';  // Plockning utförd
    INBYTE[1]='1';  //Klar
    break;

case 'm':
    // Plocka
    INBYTE[2]='u';  // Plockning utförd
    INBYTE[1]='1';  // Klar
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
    if(((sensorValue9+sensorValue10+sensorValue11+sensorValue12+sensorValue13+sensorValue14+sensorValue15+sensorValue16) / 8) > 
    ((sensorValue1+sensorValue2+sensorValue3+sensorValue4+sensorValue5+sensorValue6+sensorValue7+sensorValue8+600) / 8))
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
}
