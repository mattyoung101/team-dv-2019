#include <Wire.h>

void setup() {
  Wire.begin(0x12); // join bus on addr 0x12, ESP32 I2C slave (RL master) is 0x23
  Wire.onRequest(requestEvent);
  Serial.begin(9600);
}

void requestEvent(){
  Wire.write(0x69);
}

void loop() {
  Serial.println("epic");
//  Wire.write(0x69);
}
