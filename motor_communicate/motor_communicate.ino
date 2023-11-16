#include "BluetoothSerial.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
BluetoothSerial SerialBT;

char drive_control;
const int a1a_drive = 33;
const int a1b_drive = 32;
const int b1a_drive = 23;
const int b1b_drive = 2;

const int a1a_turn = 12;
const int a1b_turn = 14;
const int b1a_turn = 27;
const int b1b_turn = 26;

const int sleep_drive = 25;
const int sleep_turn = 13;

const int R1 = 17;
const int G1 = 16;
const int blue1 = 4;

const int R2 = 5;
const int G2 = 19;
const int B2 = 15;

const int mostrig = 18;

unsigned long previousMillis = 0;

// Variables to store the state and timing information
bool tilted = true;
unsigned long tiltStartTime = 0;
unsigned long tiltDuration = 5000;  // 5 seconds in milliseconds
int counter = 0;

// const float targetAngle = 2.0;  // Adjust as needed
// const float kp = 1.0;           // Proportional gain

// int counter=0;

void forward(int power_drive);
void backwards(int power_drive);
void stop_drive();
void right(int power_turn);
void left(int power_turn);
void light_test(int power_turn);
void stop_turn();
void kill_power();

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

  pinMode(sleep_drive, OUTPUT);
  pinMode(sleep_turn, OUTPUT);

  digitalWrite(sleep_drive,0);
  digitalWrite(sleep_turn,0);

  analogWrite(a1a_drive, 0);
  analogWrite(a1b_drive, 0);
  analogWrite(b1a_drive, 0);
  analogWrite(b1b_drive, 0);

  analogWrite(a1a_turn, 0);
  analogWrite(a1b_turn, 0);
  analogWrite(b1a_turn, 0);
  analogWrite(b1b_turn, 0);

  // Wire.begin();
  // mpu.initialize();

  // mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  // Serial.print("Filter bandwidth set to: ");
  // switch (mpu.getFilterBandwidth()) {
  // case MPU6050_BAND_260_HZ:
  //   Serial.println("260 Hz");
  //   break;
  // case MPU6050_BAND_184_HZ:
  //   Serial.println("184 Hz");
  //   break;
  // case MPU6050_BAND_94_HZ:
  //   Serial.println("94 Hz");
  //   break;
  // case MPU6050_BAND_44_HZ:
  //   Serial.println("44 Hz");
  //   break;
  // case MPU6050_BAND_21_HZ:
  //   Serial.println("21 Hz");
  //   break;
  // case MPU6050_BAND_10_HZ:
  //   Serial.println("10 Hz");
  //   break;
  // case MPU6050_BAND_5_HZ:
  //   Serial.println("5 Hz");
  //   break;
  // }

  // Serial.println("");
  // delay(100);
}


void loop() {
  // sensors_event_t a, g, temp;
  // mpu.getEvent(&a, &g, &temp);

  // /* Print out the values */
  // Serial.print("Acceleration X: ");
  // Serial.print(a.acceleration.x);
  // Serial.print(", Y: ");
  // Serial.print(a.acceleration.y);
  // Serial.print(", Z: ");
  // Serial.print(a.acceleration.z);
  // Serial.println(" m/s^2");

  // Serial.print("Rotation X: ");
  // Serial.print(g.gyro.x);
  // Serial.print(", Y: ");
  // Serial.print(g.gyro.y);
  // Serial.print(", Z: ");
  // Serial.print(g.gyro.z);
  // Serial.println(" rad/s");

  // Serial.print("Temperature: ");
  // Serial.print(temp.temperature);
  // Serial.println(" degC");

  // Serial.println("");
  String steering_controls = SerialBT.readStringUntil('\n');
  // if(counter==150)
  // {
  //   // SerialBT.end();
  //   // SerialBT.begin("Stubby");
  //   counter=0;
  // }
  // while(SerialBT.available()>0)
  // {
  //   SerialBT.read();
  // }
  // steering_controls=steering_controls.substring(2,steering_controls.length()-1);
  Serial.println(steering_controls);
  SerialBT.println(steering_controls);
  int commaIndex = steering_controls.indexOf(',');
  // SerialBT.println(commaIndex);
  if (commaIndex != -1) {
    // counter++;
    String first_part = steering_controls.substring(0, commaIndex);
    int drive_power = first_part.toInt();

    String second_part = steering_controls.substring(commaIndex + 1, steering_controls.length());
    int turn_power = second_part.toInt();
    SerialBT.println(drive_power);
    SerialBT.println(turn_power);

    if (drive_power > 0) {
      forward(drive_power);
    } else if (drive_power < 0) {
      backwards(drive_power * -1);
    } else {
      stop_drive();
    }

    if (turn_power > 0) {
      right(turn_power);
    } else if (turn_power < 0) {
      left(turn_power * -1);
    } else {
      stop_turn();
    }
  }
  kill_power();
}

void forward(int power_drive) {
  digitalWrite(sleep_drive,1);
  analogWrite(a1a_drive, power_drive);
  analogWrite(b1a_drive, power_drive);
  analogWrite(a1b_drive, 0);
  analogWrite(b1b_drive, 0);
}

void backwards(int power_drive) {
  digitalWrite(sleep_drive,1);
  analogWrite(a1a_drive, 0);
  analogWrite(b1a_drive, 0);
  analogWrite(a1b_drive, power_drive);
  analogWrite(b1b_drive, power_drive);
}

void stop_drive() {
  digitalWrite(sleep_drive,0);
  analogWrite(a1a_drive, 0);
  analogWrite(b1a_drive, 0);
  analogWrite(a1b_drive, 0);
  analogWrite(b1b_drive, 0);
}

void right(int power_turn) {
  digitalWrite(sleep_turn,1);
  analogWrite(b1a_turn, 0);
  analogWrite(a1b_turn, 0);
  analogWrite(a1a_turn, power_turn);
  analogWrite(b1b_turn, power_turn);
}

void light_test(int power_turn) {
  analogWrite(b1a_turn, power_turn);
  analogWrite(a1b_turn, power_turn);
  analogWrite(a1a_turn, power_turn);
  analogWrite(b1b_turn, power_turn);
  analogWrite(a1a_drive, power_turn);
  analogWrite(b1a_drive, power_turn);
  analogWrite(a1b_drive, power_turn);
  analogWrite(b1b_drive, power_turn);
}

void left(int power_turn) {
  digitalWrite(sleep_turn,1);
  analogWrite(b1a_turn, power_turn);
  analogWrite(a1b_turn, power_turn);
  analogWrite(a1a_turn, 0);
  analogWrite(b1b_turn, 0);
}

void stop_turn() {
  digitalWrite(sleep_turn,0);
  analogWrite(b1a_turn, 0);
  analogWrite(a1b_turn, 0);
  analogWrite(a1a_turn, 0);
  analogWrite(b1b_turn, 0);
}

void kill_power(){
  // Read the state of the tilt switch
  bool currentlyTilted = digitalRead(mostrig) == LOW;

  if (currentlyTilted != tilted) {
    // Tilt state has changed
    tilted = currentlyTilted;
    tiltStartTime = millis();  // Update the tilt start time
  }

  if (tilted && millis() - tiltStartTime >= tiltDuration) {
    // Tilted for more than 5 seconds, assert LOW on tilt switch pin
    digitalWrite(mostrig, LOW);
  } else {
    // If not tilted for more than 5 seconds, keep the tilt switch pin HIGH
    digitalWrite(mostrig, HIGH);
    tiltStartTime = millis();
  }

  // Add a small delay to avoid excessive loop iteration
  delay(10);
} 