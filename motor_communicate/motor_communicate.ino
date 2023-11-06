#include "BluetoothSerial.h"
#include <Wire.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

char drive_control;
const int a1a_drive = 4; // Forward Moving Motor 1 
const int a1b_drive = 33; //
const int b1a_drive = 32; // Forward Moving Motor 2
const int b1b_drive = 23;

const int a1a_turn = 14; // Turning Motor 1
const int a1b_turn = 25;
const int b1a_turn = 26; // Turning Motor 2
const int b1b_turn = 27;

void setup() { // Setup Bluetooth Protocol
  Serial.begin(115200);
  SerialBT.begin("Stubby"); // Bluetooth device name
  Serial.println("The device can be paired with bluetooth");
  pinMode(a1a_drive, OUTPUT); // Configure Pin Modes
  pinMode(a1b_drive, OUTPUT);
  pinMode(b1a_drive, OUTPUT);
  pinMode(b1b_drive, OUTPUT);

  pinMode(a1a_turn, OUTPUT);
  pinMode(a1b_turn, OUTPUT);
  pinMode(b1a_turn, OUTPUT);
  pinMode(b1b_turn, OUTPUT);

  analogWrite(a1a_drive, 0); // Ensure all motors are turned off
  analogWrite(a1b_drive, 0);
  analogWrite(b1a_drive, 0);
  analogWrite(b1b_drive, 0);

  analogWrite(a1a_turn, 0);
  analogWrite(a1b_turn, 0);
  analogWrite(b1a_turn, 0);
  analogWrite(b1b_turn, 0);
}

void loop() {
  drive_control = SerialBT.read(); // Send Bluetooth Serial Inputs to drive the robot
  
  
  if (drive_control == 'F'){ // Commands to move the robot forwards
    analogWrite(a1a_drive, 255);
    analogWrite(b1a_drive, 255);
    analogWrite(a1b_drive, 0);
    analogWrite(b1b_drive, 0);
    SerialBT.println('FORWARDS');

  }
  else if (drive_control == 'B'){ // Commands to move the robot backwards
    analogWrite(a1b_drive, 255);
    analogWrite(b1b_drive, 255);
    analogWrite(a1a_drive, 0);
    analogWrite(b1a_drive, 0);
    SerialBT.println('BACKWARDS');
  }
  else if (drive_control == 'R'){ // Turn the sphere to the right
    analogWrite(b1a_turn, 0);
    analogWrite(a1b_turn, 0);
    analogWrite(a1a_turn, 160);
    analogWrite(b1b_turn, 160);
    SerialBT.println('RIGHT');
  }
  else if (drive_control == 'L'){ // Turn the sphere to the left
    analogWrite(a1a_turn, 0);
    analogWrite(b1b_turn, 0);
    analogWrite(a1b_turn, 96);
    analogWrite(b1a_turn, 96);
    SerialBT.println('LEFT');
  }
  else if (drive_control == 'O'){ // Turn off all motor controls

    analogWrite(a1a_drive, 0);
    analogWrite(b1a_drive, 0);
    analogWrite(a1b_drive, 0);
    analogWrite(b1b_drive, 0);

    analogWrite(a1a_turn, 0);
    analogWrite(b1b_turn, 0);
    analogWrite(b1a_turn, 0);
    analogWrite(a1b_turn, 0);
    SerialBT.println('STOPPED');
  }

  delay(20); // Create a dummy delay time
}
