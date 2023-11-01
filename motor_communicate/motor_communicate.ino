#include "BluetoothSerial.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

char drive_control;
const int a1a_drive = 32; // Forward Moving Motor 1 
const int a1b_drive = 33; //
const int b1a_drive = 2; // Forward Moving Motor 2
const int b1b_drive = 23;

const int a1a_turn = 12; // Turning Motor 1
const int a1b_turn = 14;
const int b1a_turn = 26; // Turning Motor 2
const int b1b_turn = 27;
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Stubby"); // Bluetooth device name
  Serial.println("The device can be paired with bluetooth");
  pinMode(a1a_drive, OUTPUT);
  pinMode(a1b_drive, OUTPUT);
  pinMode(b1a_drive, OUTPUT);
  pinMode(b1b_drive, OUTPUT);

  pinMode(a1a_turn, OUTPUT);
  pinMode(a1b_turn, OUTPUT);
  pinMode(b1a_turn, OUTPUT);
  pinMode(b1b_turn, OUTPUT);

  while (!Serial)
    delay(10);

  Serial.println("Adafruit MPU6050 test!");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  drive_control = 'F'; //drive_control = SerialBT.read();
  
  
  if (drive_control == 'F'){
    digitalWrite(a1a_drive, HIGH);
    digitalWrite(b1a_drive, HIGH);
    digitalWrite(a1b_drive, LOW);
    digitalWrite(b1b_drive, LOW);
    SerialBT.println('FORWARDS');

  }
  else if (drive_control == 'B'){
    digitalWrite(a1a_drive, LOW);
    digitalWrite(b1a_drive, LOW);
    digitalWrite(a1b_drive, HIGH);
    digitalWrite(b1b_drive, HIGH);
    SerialBT.println('BACKWARDS');
  }
  else if (drive_control == 'R'){
    digitalWrite(a1a_turn, HIGH);
    digitalWrite(b1b_turn, HIGH);
    digitalWrite(b1a_turn, LOW);
    digitalWrite(a1b_turn, LOW);
    SerialBT.println('RIGHT');
  }
  else if (drive_control == 'L'){
    digitalWrite(a1a_turn, LOW);
    digitalWrite(b1b_turn, LOW);
    digitalWrite(b1a_turn, HIGH);
    digitalWrite(a1b_turn, HIGH);
    SerialBT.println('LEFT');
  }
  else if (drive_control == 'O'){
    digitalWrite(a1a_drive, LOW);
    digitalWrite(b1a_drive, LOW);
    digitalWrite(a1b_drive, LOW);
    digitalWrite(b1b_drive, LOW);

    digitalWrite(a1a_turn, LOW);
    digitalWrite(b1b_turn, LOW);
    digitalWrite(b1a_turn, LOW);
    digitalWrite(a1b_turn, LOW);
    SerialBT.println('STOPPED');
  }
  delay(1000);
  SerialBT.println(a.acceleration.z);
}
