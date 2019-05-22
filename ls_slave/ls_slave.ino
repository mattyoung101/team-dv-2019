#include <Wire.h>
#include "LightSensorArray.h"

LightSensorArray ls = LightSensorArray();

void setup() {
  Wire.begin(0x12); // join bus on address 0x12 (in slave mode)
  Wire.onRequest(requestEvent);
  Serial.begin(9600);

  // initialise LS library
  ls.init();
  ls.calibrate();
}

void loop() {
//  digitalWrite(LS_EN, LOW);
//  digitalWrite(LS_WR, LOW);
//  digitalWrite(LS_S0, HIGH);
//  digitalWrite(LS_S1, HIGH);
//  digitalWrite(LS_S2, HIGH);
//  digitalWrite(LS_S3, HIGH);
//  digitalWrite(LS_S4, LOW);
//  digitalWrite(LS_WR, HIGH);
//  Serial.print("A0: ");
//  Serial.println(analogRead(A0));
//  Serial.print("A1: ");
//  Serial.println(analogRead(A1));

  ls.read();
  ls.calculateClusters();
  ls.calculateLine();

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

