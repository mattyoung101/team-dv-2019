#include <Wire.h>

#define LS_EN 2
#define LS_S0 3
#define LS_S1 4
#define LS_S2 5
#define LS_S3 6
#define LS_S4 7
#define LS_WR 8

void setup() {
  // init I2C and UART
  Wire.begin(0x12); // join bus on address 0x12 (in slave mode)
  Wire.onRequest(requestEvent);
  Serial.begin(9600);

  // initialise the multiplexer
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(LS_EN, OUTPUT);
  pinMode(LS_S0, OUTPUT);
  pinMode(LS_S1, OUTPUT);
  pinMode(LS_S2, OUTPUT);
  pinMode(LS_S3, OUTPUT);
  pinMode(LS_S4, OUTPUT);
  pinMode(LS_WR, OUTPUT);
}

void loop() {
  digitalWrite(LS_EN, LOW);
  digitalWrite(LS_WR, LOW);
  digitalWrite(LS_S0, LOW);
  digitalWrite(LS_S1, LOW);
  digitalWrite(LS_S2, LOW);
  digitalWrite(LS_S3, LOW);
  digitalWrite(LS_S4, LOW);
  Serial.print("A0: ");
  Serial.println(analogRead(A0));
  Serial.print("A1: ");
  Serial.println(analogRead(A1));
}

void requestEvent(){
  Wire.write(0xB); // begin byte
  // write out values 
}

