//---------------------------------Libraries--------------------------
#include <Arduino.h>
#include <math.h>
#include "bluetooth.h"
#include "SoftwareSerial.h"

//------------------------------Definig variables----------------------
SoftwareSerial BTSerial(10,11); // RX , TX
String BTBYTE;  // Received signal string.
String INBYTE;  // Transmitting signal string.


// ------------------------------- function ---------------------------
String readBluetoothData()
{
int l = 0;      // l = length of recieved signal.

if(BTSerial.available())            
{
    BTBYTE=BTSerial.readString();
    l=BTBYTE.length();              // Längden på BTBYTE.
    Serial.print(BTBYTE);           // Skriver ut BTBYTE i Serial (Serialen på arduino:n).
    
}
if(Serial.available())              // 
{
    INBYTE = Serial.readString();      
    BTSerial.println(INBYTE);       // Skriver ut INBYTE i BTSerial (Bluetooth Transmitterns Serialen).
}

if(l!=0)    // New command detected.
{
    BTSerial.print(BTBYTE);
return BTBYTE;
}
else        // No new command detected.
{
return "No new command detected";
}
}


