//---------------------------------Libraries--------------------------
#include <Arduino.h>
#include <math.h>
#include "bluetooth.h"
#include "SoftwareSerial.h"

// --------Defining variables------------
char Zone;
char direction;
// ------------------------------- function ---------------------------
String readBluetoothData(String BTBYTE)
{
    // Dela upp BTBYTE meddelanden.
    Zone=BTBYTE[0];
    direction=BTBYTE[0];
    Serial.println();

    switch (Zone)
    {
    case '1':
        switch (direction)
        {
        case 'f':
            //translate_FWD();
            break;
        
        default:
            break;
        }
        return "In zone 1";
        break;
    
    case '2':
        return "In zone 2";
        break;
        
    case '3':
        return "In zone 3";
        break;

    default:
        return "AGV har tagit emot meddelandet.";
        break;
    }
    
}



