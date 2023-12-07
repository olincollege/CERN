#include "BluetoothSerial.h"
#include <Wire.h>

BluetoothSerial SerialBT;

const int a1a_drive = 4;
const int a1b_drive = 33;
const int b1a_drive = 32;
const int b1b_drive = 23;

const int a1a_turn = 14;
const int a1b_turn = 25;
const int b1a_turn = 26;
const int b1b_turn = 27;

int power_turn = 255;
void setup() {
  Serial.begin(115200);
  SerialBT.begin("Stubby");
  Serial.println("The device can be paired with Bluetooth");
  pinMode(a1a_drive, OUTPUT);
  pinMode(a1b_drive, OUTPUT);
  pinMode(b1a_drive, OUTPUT);
  pinMode(b1b_drive, OUTPUT);

  pinMode(a1a_turn, OUTPUT);
  pinMode(a1b_turn, OUTPUT);
  pinMode(b1a_turn, OUTPUT);
  pinMode(b1b_turn, OUTPUT);

  analogWrite(a1a_drive, 0);
  analogWrite(a1b_drive, 0);
  analogWrite(b1a_drive, 0);
  analogWrite(b1b_drive, 0);

  analogWrite(a1a_turn, 0);
  analogWrite(a1b_turn, 0);
  analogWrite(b1a_turn, 0);
  analogWrite(b1b_turn, 0);

}

void loop() {
  analogWrite(b1a_turn, power_turn);
  analogWrite(a1b_turn, power_turn);
  analogWrite(a1a_turn, power_turn);
  analogWrite(b1b_turn, power_turn);
  analogWrite(a1a_drive, power_turn);
  analogWrite(b1a_drive, power_turn);
  analogWrite(a1b_drive, power_turn);
  analogWrite(b1b_drive, power_turn);

  delay(2000);

  power_turn = 128;

  analogWrite(b1a_turn, power_turn);
  analogWrite(a1b_turn, power_turn);
  analogWrite(a1a_turn, power_turn);
  analogWrite(b1b_turn, power_turn);
  analogWrite(a1a_drive, power_turn);
  analogWrite(b1a_drive, power_turn);
  analogWrite(a1b_drive, power_turn);
  analogWrite(b1b_drive, power_turn);

  delay(2000);

  power_turn = 0;

  analogWrite(b1a_turn, power_turn);
  analogWrite(a1b_turn, power_turn);
  analogWrite(a1a_turn, power_turn);
  analogWrite(b1b_turn, power_turn);
  analogWrite(a1a_drive, power_turn);
  analogWrite(b1a_drive, power_turn);
  analogWrite(a1b_drive, power_turn);
  analogWrite(b1b_drive, power_turn);

  delay(2000);

}
