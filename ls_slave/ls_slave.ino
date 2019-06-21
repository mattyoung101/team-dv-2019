#include <Wire.h>
#include "LightSensorArray.h"
#include "Timer.h"

#define I2C_ON true
#define BLINK_LED 9
#define DEBUG_LED A6

Timer ledTimer(100000);
bool ledOn;

LightSensorArray ls = LightSensorArray();
float heading = 0.0f;

void setup() {
  // join bus on address 0x12 (in slave mode)
  #if I2C_ON
    Wire.begin(0x12);
    Wire.onRequest(requestEvent);
  #endif
  
  Serial.begin(9600);

  pinMode(BLINK_LED,OUTPUT);

  // initialise LS library
//  ls.init();
//  ls.calibrate();
}

void loop() {
//  ls.read();
//  ls.calculateClusters();
//  ls.calculateLine();

//  ls.updateLine((float)ls.getLineAngle(), (float)ls.getLineSize(), heading);
//  ls.lineCalc();

//  Serial.print("lineAngle: ");
//  Serial.print(ls.getLineAngle());
//  Serial.print("\t");
//  Serial.print("lineSize: ");
//  Serial.print(ls.getLineSize());
  
//  Serial.print(analogRead(A1));
//  Serial.println("fuck");

  // Blinky LEDs :D
  if(ledTimer.timeHasPassed()){
    digitalWrite(BLINK_LED,ledOn);
    ledOn = !ledOn;
  }
}

void requestEvent(){
  /*
  float inLineAngle: 2 bytes
  float inLineSize: 2 bytes
  bool inOnLine: 1 byte
  bool inLineOver: 1 byte
  float inLastAngle: 2 bytes
  = 8 bytes + 1 start byte 
  = 9 bytes total
  */
  Wire.write(0xB);
//  Wire.write(highByte((uint16_t) ls.getLineAngle()));
//  Wire.write(lowByte((uint16_t) ls.getLineAngle()));
//  Wire.write(highByte((uint16_t) ls.getLineSize()));
//  Wire.write(lowByte((uint16_t) ls.getLineSize()));
//  Wire.write(ls.isOnLine);
//  Wire.write(ls.lineOver);
  Wire.write(highByte((uint16_t) 100));
  Wire.write(lowByte((uint16_t) 100));
  Wire.write(highByte((uint16_t) 8700));
  Wire.write(lowByte((uint16_t) 8700));
  Wire.write(0);
  Wire.write(1);
}
