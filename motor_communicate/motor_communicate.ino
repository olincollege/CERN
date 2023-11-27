#include "BluetoothSerial.h" // Import libaries: Bluetooth Serial and Broadcasting, IMU Readings, and PID controls
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <PID_v1.h>

Adafruit_MPU6050 mpu; // Module for the IMU
BluetoothSerial SerialBT; // Module for Bluetooth

char drive_control;
const int a1a_drive = 33; // Pins for the drive/fast/forward moving motors
const int a1b_drive = 32;
const int b1a_drive = 2;
const int b1b_drive = 23;

const int a1a_turn = 12; // Pins for the turning/slow/pivotal motors
const int a1b_turn = 14;
const int b1a_turn = 27;
const int b1b_turn = 26;

const int sleep_drive = 25; // Motor sleep pins for the motor controllers
const int sleep_turn = 13;

const int R1 = 17; // Pins for LED 1
const int G1 = 16;
const int blue1 = 4;

const int R2 = 5; // Pins for LED 2
const int G2 = 19;
const int B2 = 15;

const int mostrig = 18; // Pin readouts for power on and power off sequence
const int tiltread = 35;

unsigned long previousMillis = 0; // Time keeping variable for power down sequence

// Variables to store the state and timing information
bool tilted = true;
bool shutdown = false;
unsigned long tiltStartTime = 0;
unsigned long tiltDuration = 5000;  // 5 seconds in milliseconds
int counter = 0;

double targetAngle = 3.0;  // Target angle (Allow for a 3 degree tolerance)
double input, output, outputDrive, outputTurn; // PID variables

double kp = 50.0; // Proportionality constant
double ki = 30.0; // Integral constant
double kd = 30.0; // Derivative constant

PID drivePID(&input, &outputDrive, &targetAngle, kp, ki, kd, DIRECT); // PID controls for the drive and turning motors
PID turnPID(&input, &outputTurn, &targetAngle, kp, ki, kd, DIRECT);

void forward(int power_drive); // These six functions are resposible for controlling the robot in whatever direction the user commands it to
void backwards(int power_drive);
void stop_drive();
void right(int power_turn);
void left(int power_turn);
void stop_turn();

void kill_power(); // Turn off the robot

void light_test(int power_turn);

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Stubby"); // When turned on, start broadcasting a Bluetooth module that everyone can see and connect to
  

  pinMode(mostrig, OUTPUT);
  digitalWrite(mostrig, 1); // Activate the mostrig pin which tells the robot to power on
  pinMode(tiltread, INPUT);
  
  pinMode(a1a_drive, OUTPUT); // Set all motor output pins as a PWM output to drive the motor
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

  pinMode(sleep_drive, OUTPUT); // Set the motor sleep controller as digital outputs and initialize to low to be off
  pinMode(sleep_turn, OUTPUT);

  digitalWrite(sleep_drive,0);
  digitalWrite(sleep_turn,0);

  pinMode(R1,OUTPUT); // Turn on the red light of LED 1 to notify the user that the robot has turned on. Then turn off
  digitalWrite(R1, 0);
  delay(500);
  digitalWrite(R1, 1);

  if (!mpu.begin()) { // Activate and connect to the on-board gyroscope and accelerometer
    SerialBT.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // Set the filtering, acceleration, and angular velocity ranges for the IMU
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  pinMode(G1,OUTPUT); /* Go through a lighting sequence to notify the user that the IMU is reading. 
                         A green light will activate telling the user that the setup is complete*/
  digitalWrite(G1, 0);
  pinMode(blue1,OUTPUT);
  digitalWrite(blue1, 1);

  pinMode(R2,OUTPUT);
  digitalWrite(R2, 1);
  drivePID.SetMode(AUTOMATIC); // Set up the PID controllers
  turnPID.SetMode(AUTOMATIC);
  pinMode(G2,OUTPUT);
  digitalWrite(G2, 1);

  // Once the green light has been turned on, the setup is complete
}


void loop() {
  /* The loop is active when the user is connected to the robot via the control application that sends
  commands over Bluetooth serial to control the robot through WASD keys */

  int loop = 0;

  String steering_controls = SerialBT.readStringUntil('\n'); 
  /* Over the serial, every user control has two comma separated integers ranging from -255 to 255 for both integers.
  The leftmost integer represents the respective motor control for the forward driving motors while the rightmost
  integer is the turning motor power. */

  int commaIndex = steering_controls.indexOf(','); // To start, first find the location of where the comma is located in the string
  if (commaIndex != -1) {
    // counter++;
    String first_part = steering_controls.substring(0, commaIndex); 
    int drive_power = first_part.toInt();
    /* The first index of the string all the way to the 
    index of the comma should be the integer of the drive motor. 
    Convert this to an integer. */

    String second_part = steering_controls.substring(commaIndex + 1, steering_controls.length());
    int turn_power = second_part.toInt();
    /* The following index of the comma all the way to the 
    final index of the string should be the integer of the turn motor. 
    Also convert this to an integer. */

    sensors_event_t a, g, temp; // IMU variables for acceleration and gyroscope
    mpu.getEvent(&a, &g, &temp); // Obtain the IMU readings for the current controls
    double input = a.acceleration.z;  // Adjust axis based on your robot's orientation

    drivePID.Compute(); // Compute PID output for both motor controllers
    turnPID.Compute();

    int drivePower = map(outputDrive, -255, 255, -100, 100); // Apply the control signals to motors by obtaining the necessary motor output voltages
    int turnPower = map(outputTurn, -255, 255, -100, 100);

    if (drive_power > 0) {
      forward(drive_power); // If the user is pressing W on their keyboard, the user would like to move forward and so the first integer is positive.
    } else if (drive_power < 0) {
      backwards(drive_power * -1); // If the user instead presses S, the user would like to move backwards which means that the motor power is negative.
    } else {
      stop_drive(); // If the user presses neither, the integer will be zero and the motor driver for the forward motors should be put to sleep.
    }

    if (turn_power > 0) {
      right(turn_power); // If the user is pressing D on their keyboard, the user would like to turn right so the second integer is positive.
    } else if (turn_power < 0) {
      left(turn_power * -1); // If the user is pressing A on their keyboard, the user would like to turn left so the second integer is negative.
    } else {
      stop_turn(); // Like the drive motors, if neither is pressed, the second integer will be zero, and motor controls should not be sent to the turn motors.
    }
  kill_power(); // At the end of the loop, run this function to see if the user would like to shut off the robot
}
}

void forward(int power_drive) { // Run the robot forward by turning on the motor controller and setting the two positive inputs at a higher voltage.
  digitalWrite(sleep_drive,1);
  analogWrite(a1a_drive, power_drive);
  analogWrite(b1a_drive, power_drive);
  analogWrite(a1b_drive, 0);
  analogWrite(b1b_drive, 0);
}

void backwards(int power_drive) { // Run the robot backwards by turning on the motor controller and setting the two negative inputs at a higher voltage.
  digitalWrite(sleep_drive,1);
  analogWrite(a1a_drive, 0);
  analogWrite(b1a_drive, 0);
  analogWrite(a1b_drive, power_drive);
  analogWrite(b1b_drive, power_drive);
}

void stop_drive() { // Stop the robot by turning off the motor controller and setting each of the drive pins to 0 volts.
  digitalWrite(sleep_drive,0);
  analogWrite(a1a_drive, 0);
  analogWrite(b1a_drive, 0);
  analogWrite(a1b_drive, 0);
  analogWrite(b1b_drive, 0);
}

void right(int power_turn) { // Turn the robot right by turning on the motor controller and setting the two positive inputs at a higher voltage.
  digitalWrite(sleep_turn,1);
  analogWrite(b1a_turn, 0);
  analogWrite(a1b_turn, 0);
  analogWrite(a1a_turn, power_turn);
  analogWrite(b1b_turn, power_turn);
}

void left(int power_turn) { // Turn the robot left by turning on the motor controller and setting the two negative inputs at a higher voltage.
  digitalWrite(sleep_turn,1);
  analogWrite(b1a_turn, power_turn);
  analogWrite(a1b_turn, power_turn);
  analogWrite(a1a_turn, 0);
  analogWrite(b1b_turn, 0);
}

void stop_turn() { // Stop the turning of the robot by turning off the motor controllers and setting each of the turn pins to 0 volts.
  digitalWrite(sleep_turn,0);
  analogWrite(b1a_turn, 0);
  analogWrite(a1b_turn, 0);
  analogWrite(a1a_turn, 0);
  analogWrite(b1b_turn, 0);
}

void kill_power(){ // Run through the sequence of  checking to see if the user would turn off the robot

  bool currentlyTilted = (digitalRead(tiltread) == HIGH);

  if (currentlyTilted) // Read the state of the tilt switch. If it is tilted approximately 60 degrees, the second green light will switch off
  {
//    digitalWrite(R2,0);
    digitalWrite(G2,1);
  }
  else
  {
//    digitalWrite(R2,1);
    digitalWrite(G2,0); // If not, the tilt switch will cause the green second light to turn on.
  }
  if (currentlyTilted != tilted) { // If the robot switches from being in a non tilted state to a tilted state, then start a timer
    // Tilt state has changed
    tilted = currentlyTilted;
    tiltStartTime = millis();  // Update the tilt start time
  }

  if (tilted && ((millis() - tiltStartTime) >= tiltDuration)) { 
    /* If the amount of time the robot has been tilted for is greater than or equal to 5 seconds, 
    then run the shutoff command. */

    shutdown = true; // Tilted for more than 5 seconds, assert LOW on tilt switch pin
    digitalWrite(G1,1);
    digitalWrite(blue1, 0); // Flash the blue light to notify the user that the robot is about to turn off
    delay(500);
    digitalWrite(blue1, 1);
  } 
  else if(!tilted && shutdown && ((millis() - tiltStartTime) >= tiltDuration/5)) // Once the robot has been tilted for more than 5 seconds, the robot has to be tilted up again to kill the power
  {
    digitalWrite(mostrig, LOW);
  }


  // Small delay to avoid excessive loop iteration
  delay(10);
} 
