#include <Wire.h>
#include "Config.h"
#include "LightSensorArray.h"

#define I2C_ON true

void setup() {
  // put your setup code here, to run once:
  #if I2C_ON
    Wire.begin(0x12);
    Wire.onRequest(requestEvent);
  #endif
  
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

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
