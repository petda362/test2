#ifndef bluetooth_h
#define bluetooth_h
#include <Arduino.h>
#include <math.h>
#include "SoftwareSerial.h"

// ------------Function declaration--------------------------------

String readBluetoothData(String BTBYTE, int PWM);

String Instructions(char inst, int PWM, String INBYTE);

String Plocka(char inst, int PWM, String INBYTE);

String Tejpbitar(char inst, int PWM, String INBYTE);
 
void readIRData();







#endif