#include <Wire.h>
#include "LightSensorArray.h"
#include "Timer.h"

#define I2C_ON true
#define BLINK_LED 9
#define DEBUG_LED A6
#define V_BAT A7
#define V_REF 5.027
#define R1 1000000
#define R2 300000
#define V_BAT_OFFSET 0.333
#define V_BAT_MIN 11.1

Timer ledTimer(500000);
Timer lowVoltageTimer(1000);
bool ledOn;

LightSensorArray ls = LightSensorArray();
float heading = 0.0f;

float inputVoltage;
float batteryVoltage;

void setup() {
  // join bus on address 0x12 (in slave mode)
  #if I2C_ON
    Wire.begin(0x12);
    Wire.onRequest(requestEvent);
  #endif
  
  Serial.begin(9600);

  pinMode(BLINK_LED,OUTPUT);
  pinMode(V_BAT,INPUT);

  // initialise LS library
  ls.init();
  ls.calibrate();
}

double lerp(int fromValue, int toValue, int progress){
    return fromValue + (toValue - fromValue) * progress;
}

void loop() {
  ls.read();
  ls.calculateClusters();
  ls.calculateLine();

  ls.updateLine((float)ls.getLineAngle(), (float)ls.getLineSize(), heading);
  ls.lineCalc();

  inputVoltage = V_REF * (double)analogRead(V_BAT)/1023;
  batteryVoltage = (float)((inputVoltage * (R1 + R2)) / R2) + V_BAT_OFFSET;

//  Serial.print(batteryVoltage);

//  Serial.print("lineAngle: ");
//  Serial.print(ls.getLineAngle());
//  Serial.print("\t");
//  Serial.print("lineSize: ");
//  Serial.print(ls.getLineSize());
//  Serial.print("\t");
//  Serial.print("isOnLine: ");
//  Serial.print(ls.isOnLine);
//  Serial.print("\t");
//  Serial.print("lineOver: ");
//  Serial.print(ls.lineOver);

  // LED Stuff
  if(batteryVoltage < V_BAT_MIN){
    if(lowVoltageTimer.timeHasPassed()){
      digitalWrite(BLINK_LED, ledOn);
      ledOn = !ledOn;
    }
  } else {
    if(ledTimer.timeHasPassed()){
      digitalWrite(BLINK_LED, ledOn);
      ledOn = !ledOn;
    }
  }


//  digitalWrite(DEBUG_LED, ls.isOnLine || ls.lineOver);

  Serial.println();
}

void requestEvent(){
  /*
  float inLineAngle: 2 bytes
  float inLineSize: 2 bytes
  bool inOnLine: 1 byte
  bool inLineOver: 1 byte
  float inLastAngle: 2 bytes
  float batteryVoltage: 2 bytes
  = 10 bytes + 1 start byte 
  = 11 bytes total
  */
  Wire.write(0xB);
  Wire.write(highByte((uint16_t) (ls.getLineAngle() * 100.0)));
  Wire.write(lowByte((uint16_t) (ls.getLineAngle() * 100.0)));
  Wire.write(highByte((uint16_t) (ls.getLineSize() * 100.0)));
  Wire.write(lowByte((uint16_t) (ls.getLineSize() * 100.0)));
  Wire.write(ls.isOnLine);
  Wire.write(ls.lineOver);
  Wire.write(highByte((uint16_t) (batteryVoltage * 100.0)));
  Wire.write(lowByte((uint16_t) (batteryVoltage * 100.0)));
}
