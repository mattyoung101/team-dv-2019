#include <Wire.h>

void setup() {
  Wire.begin(0x12); // join bus on addr 0x12, ESP32 I2C slave (RL master) is 0x23
  Wire.onRequest(requestEvent);
  Serial.begin(9600);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
}

void requestEvent(){
  Wire.write(0x69);
}

void loop() {
  Serial.println(analogRead(A1));
  Serial.println(analogRead(A2));
  Serial.println("Kill me");
  delay(100);
//  Wire.write(0x69);
}
