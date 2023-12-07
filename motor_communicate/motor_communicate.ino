#include "BluetoothSerial.h"  // Import libaries: Bluetooth Serial and Broadcasting, IMU Readings, and PID controls
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <PID_v1.h>

Adafruit_MPU6050 mpu;      // Module for the IMU
BluetoothSerial SerialBT;  // Module for Bluetooth

char drive_control;
const int a1a_turn = 12;  // Pins for the turning/slow/pivotal motors
const int a1b_turn = 14; //girthy 2,23,32,33
const int b1a_turn = 27; //stubby 12,14,27,26
const int b1b_turn = 26;

const int a1a_drive = 33; // Pins for the drive/fast/forward moving motors 
const int a1b_drive = 32; //girthy 26,27,14,12
const int b1a_drive = 2; //stubby 33,32,2,23
const int b1b_drive = 23;

const int sleep_drive = 25;  // Motor sleep pins for the motor controllers
const int sleep_turn = 13;   //girthy 13,25
                             //stubby 25,13

const int R1 = 17;  // Pins for LED 1
const int G1 = 16;
const int blue1 = 4;

const int R2 = 5;  // Pins for LED 2
const int G2 = 19;
const int B2 = 15;

const int mostrig = 18;  // Pin readouts for power on and power off sequence
const int tiltread = 35;

const int power_pin = 34;
const int MPU6050_ADDR = 0x68;  // MPU6050 I2C address
const float ACC_SCALE = 16384.0; // MPU6050 accelerometer sensitivity: 16384 LSB/G
const float GYRO_SCALE = 131.0;  // MPU6050 gyroscope sensitivity: 131 LSB/°/s

unsigned long previousMillis = 0;  // Time keeping variable for power down sequence

const uint32_t IDLE_MAX = 300000;
uint32_t last_drive_input;
uint32_t t;
double batt_voltage;

//LED Stuff
bool currentlyTilted;
bool wasTilted=false;
int leftledcode[5]={0,0,0,0,0};
int rightledcode[5]={0,0,0,0,0};
int leftledcounters[2]={0,0};
int rightledcounters[2]={0,0};
int drive_power=0;
int turn_power=0;

// Variables to store the state and timing information
bool tilted = true;
bool shutdown = false;
unsigned long tiltStartTime = 0;
unsigned long tiltDuration = 5000;  // 5 seconds in milliseconds
int counter = 0;

void forward(int power_drive);  // These six functions are resposible for controlling the robot in whatever direction the user commands it to
void backwards(int power_drive);
void stop_drive();
void right(int power_turn);
void left(int power_turn);
void stop_turn();

void kill_power();  // Turn off the robot

void light_test(int power_turn);

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Stubby");  // When turned on, start broadcasting a Bluetooth module that everyone can see and connect to

  delay(500);
  batt_voltage=(double)analogRead(power_pin)*2/1158;
  pinMode(mostrig, OUTPUT);
  if(batt_voltage>3.3)
  {
    digitalWrite(mostrig, 1);  // Activate the mostrig pin which tells the robot to power on
  }
  else
  {
    for(int i=0; i<10; i++)
    {
      analogWrite(R2, 255);
      analogWrite(R1, 0);
      delay(20);
      analogWrite(R1, 255);
      analogWrite(R2, 0);
      delay(20);
    }
  }
  pinMode(tiltread, INPUT);

  pinMode(a1a_drive, OUTPUT);  // Set all motor output pins as a PWM output to drive the motor
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

  pinMode(sleep_drive, OUTPUT);  // Set the motor sleep controller as digital outputs and initialize to low to be off
  pinMode(sleep_turn, OUTPUT);

  digitalWrite(sleep_drive, 0);
  digitalWrite(sleep_turn, 0);

  pinMode(R1,OUTPUT); /* Go through a lighting sequence to notify the user that the IMU is reading. 
                         A green light will activate telling the user that the setup is complete*/
  pinMode(G1,OUTPUT);
  pinMode(blue1,OUTPUT);
  pinMode(R2,OUTPUT);
  pinMode(G2,OUTPUT);
  pinMode(B2,OUTPUT);
  
  analogWrite(G1, 0);
  analogWrite(blue1, 255);
  Wire.begin(); // Initialize comunication
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Clear the sleep bit
  Wire.endTransmission(true);

  last_drive_input=millis();

  analogWrite(R2, 255);
  analogWrite(G2, 255);

  // Once the green light has been turned on, the setup is complete
}


void loop() {
  /* The loop is active when the user is connected to the robot via the control application that sends
  commands over Bluetooth serial to control the robot through WASD keys */

  String steering_controls = SerialBT.readStringUntil('\n');
  /* Over the serial, every user control has two comma separated integers ranging from -255 to 255 for both integers.
  The leftmost integer represents the respective motor control for the forward driving motors while the rightmost
  integer is the turning motor power. */

  parse_input(steering_controls);
  float y_accel = 0.0;

  for (int record = 0; record < 50; record++){
    float accelX_G, accelY_G, accelZ_G, gyroX_rad, gyroY_rad, gyroZ_rad;
    readMPU6050Data(accelX_G, accelY_G, accelZ_G, gyroX_rad, gyroY_rad, gyroZ_rad);
    y_accel = y_accel + accelY_G;
  }

  y_accel = y_accel/50;
  if ((drive_power!=0) && (abs(y_accel) > 0.15)){
    if (y_accel > 0){
      turn_power = turn_power - 60 - 120*y_accel;
      if (turn_power < -255){
        turn_power = -255;
      }
    } else if (y_accel < 0){
      turn_power = turn_power + 60 + 120*y_accel;
      if (turn_power > 255){
        turn_power = 255;
      }
    }

  }


  if (drive_power > 0) {
    forward(drive_power);  // If the user is pressing W on their keyboard, the user would like to move forward and so the first integer is positive.
  } else if (drive_power < 0) {
    backwards(drive_power * -1);  // If the user instead presses S, the user would like to move backwards which means that the motor power is negative.
  } else {
    stop_drive();  // If the user presses neither, the integer will be zero and the motor driver for the forward motors should be put to sleep.
  }

  if (turn_power > 0) {
    right(turn_power);  // If the user is pressing D on their keyboard, the user would like to turn right so the second integer is positive.
  } else if (turn_power < 0) {
    left(turn_power * -1);  // If the user is pressing A on their keyboard, the user would like to turn left so the second integer is negative.
  } else {
    stop_turn();  // Like the drive motors, if neither is pressed, the second integer will be zero, and motor controls should not be sent to the turn motors.
  }
  led_control();
  batt_voltage=(double)analogRead(power_pin)*2/1158;
  SerialBT.println(batt_voltage);
  kill_power();  // At the end of the loop, run this function to see if the user would like to shut off the robot
}

/*
** Returns a boolean value that indicates whether the current time, t, is later than some prior
** time, t0, plus a given interval, dt.  The condition accounts for timer overflow / wraparound.
*/
bool it_is_time(uint32_t t, uint32_t t0, uint32_t dt) {
  return ((t >= t0) && (t - t0 >= dt)) ||         // The first disjunct handles the normal case
         ((t < t0) && (t + (~t0) + 1 >= dt));  //   while the second handles the overflow case
}

void readMPU6050Data(float &accelX_G, float &accelY_G, float &accelZ_G, float &gyroX_rad, float &gyroY_rad, float &gyroZ_rad) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);  // Request 14 bytes of data
  int16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ;

  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();

  Wire.read();  // Skip 2 bytes of temperature data

  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();

  accelX_G = accelX / ACC_SCALE;
  accelY_G = accelY / ACC_SCALE;
  accelZ_G = accelZ / ACC_SCALE;

  // Normalize gyroscope data to radians per second
  gyroX_rad = gyroX / GYRO_SCALE * 57.2958;
  gyroY_rad = gyroY / GYRO_SCALE * 57.2958;
  gyroZ_rad = gyroZ / GYRO_SCALE * 57.2958;
}

void parse_input(String input)
{
  int currentindex=-1;
  int numbercount=1;
  while(true)
  {
    int nextindex=input.indexOf(',',currentindex+1);
    if(nextindex==-1)
    {
      if(numbercount==1)
      {
        break;
      }
      String sub = input.substring(currentindex+1, input.length()); 
      if(sub.equals("shutdown"))
      {
        initiate_shutdown();
        break;
      }
      assign_input(sub.toInt(),numbercount);
      break;
    }
    String sub = input.substring(currentindex+1, nextindex); 
    assign_input(sub.toInt(),numbercount);
    numbercount++;
    currentindex=nextindex;
  }
}

void assign_input(int value, int numbercount)
{
  switch(numbercount)
  {
    case 1: //drive power
      drive_power=value;
      break;
    case 2: //turn power
      turn_power=value;
      break;
    default:
      if(numbercount<8)
      {
        leftledcode[(numbercount-3)%5]=value;
      }
      else
      {
        rightledcode[(numbercount-3)%5]=value;
      }
      break;
  }
}

void led_control()
{
//  int leftledcode[5];
//int rightledcode[5];
//int leftledcounters[2];
//int rightledcounters[2];
//const int R1 = 17; // Pins for LED 1
//const int G1 = 16;
//const int blue1 = 4;
//
//const int R2 = 5; // Pins for LED 2
//const int G2 = 19;
//const int B2 = 15;
  if(!currentlyTilted)
  {
    wasTilted=false;
    //Left LED
    if(leftledcounters[0]<leftledcode[3])
    {
      analogWrite(R1,255-leftledcode[0]);
      analogWrite(G1,255-leftledcode[1]);
      analogWrite(blue1,255-leftledcode[2]);
      leftledcounters[0]++;
    }
    else if (leftledcounters[1]<leftledcode[4])
    {
      analogWrite(R1,255);
      analogWrite(G1,255);
      analogWrite(blue1,255);
      leftledcounters[1]++;
    }
    else
    {
      leftledcounters[0]=0;
      leftledcounters[1]=0;
    }
    //Right LED
    if(rightledcounters[0]<rightledcode[3])
    {
      analogWrite(R2,255-rightledcode[0]);
      analogWrite(G2,255-rightledcode[1]);
      analogWrite(B2,255-rightledcode[2]);
      rightledcounters[0]++;
    }
    else if (rightledcounters[1]<rightledcode[4])
    {
      analogWrite(R2,255);
      analogWrite(G2,255);
      analogWrite(B2,255);
      rightledcounters[1]++;
    }
    else
    {
      rightledcounters[0]=0;
      rightledcounters[1]=0;
    }
  }
  else
  {
    if(!wasTilted)
    {
      analogWrite(R1,255);
      analogWrite(G1,255);
      analogWrite(blue1,255);
      analogWrite(R2,255);
      analogWrite(G2,255);
      analogWrite(B2,255);
    }
    wasTilted=true;
  }
}

void forward(int power_drive) {  // Run the robot forward by turning on the motor controller and setting the two positive inputs at a higher voltage.
  digitalWrite(sleep_drive, 1);
  analogWrite(a1a_drive, power_drive);
  analogWrite(b1a_drive, power_drive);
  analogWrite(a1b_drive, 0);
  analogWrite(b1b_drive, 0);
  last_drive_input=millis();
}

void backwards(int power_drive) {  // Run the robot backwards by turning on the motor controller and setting the two negative inputs at a higher voltage.
  digitalWrite(sleep_drive, 1);
  analogWrite(a1a_drive, 0);
  analogWrite(b1a_drive, 0);
  analogWrite(a1b_drive, power_drive);
  analogWrite(b1b_drive, power_drive);
  last_drive_input=millis();
}

void stop_drive() {  // Stop the robot by turning off the motor controller and setting each of the drive pins to 0 volts.
  digitalWrite(sleep_drive, 0);
  analogWrite(a1a_drive, 0);
  analogWrite(b1a_drive, 0);
  analogWrite(a1b_drive, 0);
  analogWrite(b1b_drive, 0);
}

void right(int power_turn) {  // Turn the robot right by turning on the motor controller and setting the two positive inputs at a higher voltage.
  digitalWrite(sleep_turn, 1);
  analogWrite(b1a_turn, 0);
  analogWrite(a1b_turn, 0);
  analogWrite(a1a_turn, power_turn);
  analogWrite(b1b_turn, power_turn);
}

void left(int power_turn) {  // Turn the robot left by turning on the motor controller and setting the two negative inputs at a higher voltage.
  digitalWrite(sleep_turn, 1);
  analogWrite(b1a_turn, power_turn);
  analogWrite(a1b_turn, power_turn);
  analogWrite(a1a_turn, 0);
  analogWrite(b1b_turn, 0);
}

void stop_turn() {  // Stop the turning of the robot by turning off the motor controllers and setting each of the turn pins to 0 volts.
  digitalWrite(sleep_turn, 0);
  analogWrite(b1a_turn, 0);
  analogWrite(a1b_turn, 0);
  analogWrite(a1a_turn, 0);
  analogWrite(b1b_turn, 0);
}

void kill_power(){ // Run through the sequence of  checking to see if the user would turn off the robot

  currentlyTilted = (digitalRead(tiltread) == HIGH);

  if (currentlyTilted) // Read the state of the tilt switch. If it is tilted approximately 60 degrees, the second green light will switch on
  {
//    digitalWrite(R2,0);
    analogWrite(G2,0);
  }
  else
  {
//    digitalWrite(R2,1);
//    digitalWrite(G2,0); // If not, the tilt switch will cause the green second light to turn on.
  }
  if (currentlyTilted != tilted) { // If the robot switches from being in a non tilted state to a tilted state, then start a timer
    // Tilt state has changed
    tilted = currentlyTilted;
    tiltStartTime = millis();  // Update the tilt start time
  }

  if (tilted && ((millis() - tiltStartTime) >= tiltDuration)) { 
    /* If the amount of time the robot has been tilted for is greater than or equal to 5 seconds, 
    then run the shutoff command. */
    initiate_shutdown();

  } 
  else if(!tilted && shutdown && ((millis() - tiltStartTime) >= tiltDuration/5)) // Once the robot has been tilted for more than 5 seconds, the robot has to be tilted up again to kill the power
  {
    digitalWrite(mostrig, LOW);
  }
  if (it_is_time(millis(), last_drive_input, IDLE_MAX))
  {
    initiate_shutdown();
  }

  // Small delay to avoid excessive loop iteration
  delay(10);
} 

void initiate_shutdown()
{
    shutdown = true; // Tilted for more than 5 seconds, assert LOW on tilt switch pin
//    digitalWrite(G1,1);
    analogWrite(blue1, 0); // Flash the blue light to notify the user that the robot is about to turn off
    delay(500);
    analogWrite(blue1, 255);
}
