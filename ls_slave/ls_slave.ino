#include <Wire.h>
#include "LightSensorArray.h"

LightSensorArray ls = LightSensorArray();

float heading;

void setup() {
//  Wire.begin(0x12); // join bus on address 0x12 (in slave mode)
//  Wire.onRequest(requestEvent);
  Serial.begin(9600);

  // initialise LS library
  ls.init();
  ls.calibrate();
}

void loop() {
//  digitalWrite(2, LOW);
//  digitalWrite(8, LOW);
//  digitalWrite(3, LOW);
//  digitalWrite(4, LOW);
//  digitalWrite(5, HIGH);
//  digitalWrite(6, HIGH);
//  digitalWrite(7, LOW);
//  digitalWrite(8, HIGH);
//  Serial.print("A0: ");
//  Serial.print(analogRead(A0));
//  Serial.print("\t");
//  Serial.print("A1: ");
//  Serial.println(analogRead(A1));

  ls.read();
  ls.calculateClusters();
  ls.calculateLine();

  ls.updateLine((float)ls.getLineAngle(), (float)ls.getLineSize(), heading);
  ls.lineCalc();

//  Serial.print("lineAngle: ");
//  Serial.print(ls.getLineAngle());
//  Serial.print("\t");
//  Serial.print("lineSize: ");
//  Serial.print(ls.getLineSize());
}

void requestEvent(){
  Wire.write(0xB);
  // TODO return the latest LS reading we got here (instant response time then) - clusters are updated in loop()
}

