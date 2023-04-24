/*
MIT License

Copyright (c) 2023 jpbaehr4308

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <PID_v1.h> //https://github.com/br3ttb/Arduino-PID-Library
#include <VL53L0X.h> //https://github.com/pololu/vl53l0x-arduino
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/ElectronicCats/mpu6050
#include <Servo.h> //Arduino built in servo library

#include <ros.h>  //Libraries needed for ROS
#include <std_msgs/Int8.h> //Looking for INT from ros.h
#include <std_msgs/Int32.h> //Looking for INT from ros.h
ros::NodeHandle nh; //Node handling


#include <Wire.h>

// Define MPU objects and values //Written from MPU6050 Library
MPU6050 mpu;
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

int s = -1;

void stateCb(const std_msgs::Int8& msg)
{
  s = msg.data; //Set data from Jetson to global variable
}

ros::Subscriber<std_msgs::Int8> sub("data", &stateCb);
//ros::Publisher pub("detect", &visionState);
std_msgs::Int32 visionState;
std_msgs::Int32 approach;

ros::Publisher pub("detect", &visionState);
ros::Publisher pub2("approach", &approach);
// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
double yaw = ypr[0];
float avyaw[10];

#define INTERRUPT_PIN 18

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high //Written from MPU6050 Library
void dmpDataReady() {
  mpuInterrupt = true;
}

int xDist;
int yDist;

float yawtarget;

int state;

//Defining Pins on Mega:

//Motor Driver Enable Pins
#define LMotorEN_1 48
#define LMotorEN_2 46
#define RMotorEN_1 44
#define RMotorEN_2 44


//Motor Directions
#define RMotorBWD 4
#define RMotorFWD 5

#define LMotorBWD 6  //
#define LMotorFWD 7

//Motor Encoder Pins
#define LEncoderA 2   //Left Encoder Phase A
#define LEncoderB 52  //Left Encoder Phase B
#define REncoderA 3   //Right Encoder Phase A
#define REncoderB 50  //Right Encoder Phase A

//Servo Pins
#define servoTPin 10  //Top Servo Pin
#define servoLPin 9   //Left Servo Pin
#define servoRPin 8   //Right Servo Pin

//Defining States of Robot - set int Robot_State to one of these states to progress FSM/Robot sequence
//ex: Robot_State = Center_Beam; - puts our bot into "center_beam" part of code

int Robot_Sequence = 0;

//Servo Motor Definitions

Servo servoT;  //Top Servo
Servo servoL;  //Left Servo
Servo servoR;  //Right Servo*/

volatile int LENCA = 0;  //Left Encoder A count
volatile int RENCA = 0;  //Right Encoder A count
int LENCAx;
int RENCAx;
int LENCAy;
int RENCAy;
int LENCAsy;
int RENCAsy;
int LENCAfy;
int RENCAfy;
int prevLENCA;
int prevRENCA;
int turnLENCA = 0;
int turnRENCA = 0;
int i;

int interval = 150;

int vfilt = 0;


int SG90minUs = 1000;
int SG90maxUs = 2000;

int DS3225minUs = 500;
int DS3225maxUs = 2500;



// Distance Sensor Initialization
VL53L0X sensor;
double angle;
int angleT;
int angleL;
int angleR;

//Drive Motor Definitions
double targetdown = -50;
double targetup = 70;

double targetforward = 110;

double targetangle;
float prevangle;
int anglestate = 8;

double Lup_SPEED;
double Rup_SPEED;

double Lupangle_SPEED;
double Rupangle_SPEED;

double Ldown_SPEED;
double Rdown_SPEED;

double Lforward_SPEED;
double Rforward_SPEED;
double Lbackward_SPEED;
double Rbackward_SPEED;

double rpm_L;
double rpm_R;
unsigned int currentMillis;
unsigned int previousMillis;
unsigned int statepreviousMillis;
unsigned int freezeMillis;

int flag;
int turnFlag = 0;

int climb_LENCA;
int climb_RENCA;

int climb_LENCA2;
int climb_RENCA2;

int adjust_LENCA;
int adjust_RENCA;
int servoAngle;
int servoAngle2;
int valueDetect;
int approachValue;

PID forwardLPID(&rpm_L, &Lforward_SPEED, &targetforward, .5, .9, 0, DIRECT);
PID forwardRPID(&rpm_R, &Rforward_SPEED, &targetforward, .5, .9, 0, DIRECT);

PID mydropLPID(&rpm_L, &Ldown_SPEED, &targetdown, .9, .9, 0, DIRECT);  //.85
PID mydropRPID(&rpm_R, &Rdown_SPEED, &targetdown, .9, .9, 0, DIRECT);  //.85

PID myupangleLPID(&yaw, &Lupangle_SPEED, &targetangle, 1, .9, 0, DIRECT);  //.85
PID myupangleRPID(&yaw, &Rupangle_SPEED, &targetangle, 1, .9, 0, DIRECT);  //.85

PID myupLPID(&rpm_L, &Lup_SPEED, &targetup, 1, .9, 0, DIRECT);  //.85
PID myupRPID(&rpm_R, &Rup_SPEED, &targetup, 1, .9, 0, DIRECT);  //.85

void setup() {
  //ROS
  nh.initNode(); //Initialize node
  nh.subscribe(sub); //Setup subsriber "sub"
  nh.advertise(pub);
  nh.advertise(pub2);
  nh.getHardware()->setBaud(115200); //Setting up baudrate

  mydropLPID.SetMode(AUTOMATIC);
  mydropRPID.SetMode(AUTOMATIC);
  mydropLPID.SetSampleTime(150);
  mydropRPID.SetSampleTime(150);

  forwardLPID.SetMode(AUTOMATIC);
  forwardRPID.SetMode(AUTOMATIC);
  forwardLPID.SetSampleTime(150);
  forwardRPID.SetSampleTime(150);
  forwardLPID.SetOutputLimits(80, 120);
  forwardRPID.SetOutputLimits(80, 120);

  myupLPID.SetMode(AUTOMATIC);
  myupRPID.SetMode(AUTOMATIC);
  myupLPID.SetSampleTime(150);
  myupRPID.SetSampleTime(150);
  myupLPID.SetOutputLimits(180, 230);
  myupRPID.SetOutputLimits(180, 230);

  myupangleLPID.SetMode(AUTOMATIC);
  myupangleRPID.SetMode(AUTOMATIC);
  myupangleLPID.SetSampleTime(150);
  myupangleRPID.SetSampleTime(150);
  myupangleLPID.SetOutputLimits(200, 230);
  myupangleRPID.SetOutputLimits(200, 230);

  //MPU setup

  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);  // 400kHz I2C clock. Comment this line if having compilation difficulties //Written from MPU6050 Library

  Serial.begin(115200);
  while (!Serial)
    ;  // wait for Leonardo enumeration, others continue immediately

  // initialize device
  mpu.initialize(); //Written from MPU6050 Library
  pinMode(INTERRUPT_PIN, INPUT); //Written from MPU6050 Library


  // load and configure the DMP
  devStatus = mpu.dmpInitialize(); //Written from MPU6050 Library

  // supply your own gyro offsets here, scaled for min sensitivity //Written from MPU6050 Library
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip

  if (devStatus == 0) {   //Written from MPU6050 Library
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }


  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  //sensor.setMeasurementTimingBudget(200000);
  sensor.startContinuous(100); //Written by VL53L0X library


  //Motor Setup
  pinMode(RMotorBWD, OUTPUT);
  pinMode(RMotorFWD, OUTPUT);
  pinMode(LMotorBWD, OUTPUT);
  pinMode(LMotorFWD, OUTPUT);

  pinMode(LMotorEN_1, OUTPUT);
  pinMode(LMotorEN_2, OUTPUT);
  pinMode(RMotorEN_1, OUTPUT);
  pinMode(RMotorEN_2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(LEncoderA), leftENCA, RISING);
  attachInterrupt(digitalPinToInterrupt(REncoderA), rightENCA, RISING);
  pinMode(LEncoderB, INPUT);
  pinMode(REncoderB, INPUT);

  //Servo Setup

  servoT.attach(servoTPin, SG90minUs, SG90maxUs);
  servoL.attach(servoLPin, DS3225minUs, DS3225maxUs);
  servoR.attach(servoRPin, DS3225minUs, DS3225maxUs);

  servoT.write(175);
  servoL.write(50);
  servoR.write(0);

  //PID Controller Setup
  statepreviousMillis = millis();
}

void loop() {
  //servoT.write(175);
  // ROS
  nh.spinOnce(); //Place in Loop

  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) { //takes RPM reading every 150 ms
    previousMillis = currentMillis;
    double LENCA_meas = LENCA - prevLENCA;
    double RENCA_meas = RENCA - prevRENCA;
    prevLENCA = LENCA;
    prevRENCA = RENCA;
    rpm_L = (double)((LENCA_meas / 374) * 400);
    rpm_R = (double)((RENCA_meas / 374) * 400);
    get_MPU_data(); //pulls pitch yaw roll values from gy 521
  }

  //PID computation Loop
  nh.spinOnce();
  mydropLPID.Compute();
  mydropRPID.Compute();
  myupLPID.Compute();
  myupRPID.Compute();
  myupangleRPID.Compute();
  myupangleLPID.Compute();
  forwardLPID.Compute();
  forwardRPID.Compute();

  //Main Loop

  switch (Robot_Sequence) {
    case 0: //Wait for bootup to occur
      //servoL.write(50);
      //servoR.write(0);
      servoT.write(10);
      valueDetect = 0;
      approachValue = 0;
      visionState.data = valueDetect;
      approach.data = approachValue;
      if (s >= 0) {
        pub.publish(&visionState);
        pub2.publish(&approach);
        Robot_Sequence = 1;
      }
      break;
    case 1:   //Find Beam
      servoT.write(10);
      if (s == 2) {
        digitalWrite(RMotorEN_1, HIGH);
        digitalWrite(RMotorEN_2, HIGH);
        digitalWrite(LMotorEN_1, HIGH);
        digitalWrite(LMotorEN_2, HIGH);

        analogWrite(RMotorFWD, Rforward_SPEED);
        analogWrite(LMotorFWD, Lforward_SPEED);
      }
      if (s == 0) {
        digitalWrite(RMotorEN_1, HIGH);
        digitalWrite(RMotorEN_2, HIGH);
        digitalWrite(LMotorEN_1, HIGH);
        digitalWrite(LMotorEN_2, HIGH);

        analogWrite(RMotorFWD, Rforward_SPEED);
        analogWrite(LMotorFWD, Lforward_SPEED);
      }
      if (s == 1) {
        Robot_Sequence = 2;
        digitalWrite(RMotorEN_1, LOW);
        digitalWrite(RMotorEN_2, LOW);
        digitalWrite(LMotorEN_1, LOW);
        digitalWrite(LMotorEN_2, LOW);

        analogWrite(RMotorFWD, 0);
        analogWrite(LMotorFWD, 0);
        statepreviousMillis = currentMillis;
        LENCAx = LENCA;
        RENCAx = RENCA;
      }
      break;
    case 2:   //Wait 2 Seconds
      if (currentMillis - statepreviousMillis > 2000) {
        Robot_Sequence = 3;
        turnLENCA = LENCA;
        turnRENCA = RENCA;
      }
      break;
    case 3: //Backup to center
      if ((turnLENCA - LENCA > 400) && (turnLENCA - RENCA > 400)) {
        digitalWrite(RMotorEN_1, LOW);
        digitalWrite(RMotorEN_2, LOW);
        digitalWrite(LMotorEN_1, LOW);
        digitalWrite(LMotorEN_2, LOW);

        analogWrite(RMotorFWD, 0);
        analogWrite(LMotorFWD, 0);
        analogWrite(RMotorBWD, 0);
        analogWrite(LMotorBWD, 0);
        statepreviousMillis = currentMillis;
        Robot_Sequence = 4;
      }
      else if ((turnLENCA - LENCA < 400) && (turnLENCA - RENCA < 400)) {
        digitalWrite(RMotorEN_1, HIGH);
        digitalWrite(RMotorEN_2, HIGH);
        digitalWrite(LMotorEN_1, HIGH);
        digitalWrite(LMotorEN_2, HIGH);

        analogWrite(RMotorBWD, 100);
        analogWrite(LMotorBWD, 100);
      }

      break;

    case 4: //Wait 2 seconds
      servoT.write(180);
      if ((currentMillis - statepreviousMillis > 2000)) {
        Robot_Sequence = 5;
        turnFlag = 0;
        turnLENCA = LENCA;
        turnRENCA = RENCA;
      }
      break;
    case 5:
      if (LENCA - turnLENCA < 400) {
        turnFlag = 1;
      }
      if (turnFlag == 1)
      {
        if (s == 0 || s == 2) {
          digitalWrite(RMotorEN_1, HIGH);
          digitalWrite(RMotorEN_2, HIGH);
          digitalWrite(LMotorEN_1, HIGH);
          digitalWrite(LMotorEN_2, HIGH);

          analogWrite(RMotorBWD, 0);
          analogWrite(LMotorBWD, 0);
          analogWrite(RMotorFWD, 0);
          analogWrite(LMotorFWD, 60);
        }

        if (s == 1) {
          Robot_Sequence = 6;
          statepreviousMillis = currentMillis;
          digitalWrite(RMotorEN_1, LOW);
          digitalWrite(RMotorEN_2, LOW);
          digitalWrite(LMotorEN_1, LOW);
          digitalWrite(LMotorEN_2, LOW);

          analogWrite(RMotorFWD, 0);
          analogWrite(LMotorFWD, 0);
          analogWrite(RMotorBWD, 0);
          analogWrite(LMotorBWD, 0);
        }
      }
      if (turnFlag == 0) {
        digitalWrite(RMotorEN_1, HIGH);
        digitalWrite(RMotorEN_2, HIGH);
        digitalWrite(LMotorEN_1, HIGH);
        digitalWrite(LMotorEN_2, HIGH);

        analogWrite(RMotorBWD, 0);
        analogWrite(LMotorBWD, 0);
        analogWrite(RMotorFWD, 0);
        analogWrite(LMotorFWD, 60);
      }
      break;
    case 6:
      if (currentMillis - statepreviousMillis > 2000) {
        Robot_Sequence = 7;
        approachValue = 1;
        approach.data = approachValue;
        pub2.publish(&approach);
        LENCAsy = LENCA; // LENCAground = LENCAf - LENCAs
        RENCAsy = RENCA;
      }
      break;
    case 7://Approach the beam
      if (sensor.readRangeContinuousMillimeters() <= 120) {
        Robot_Sequence = 8;
        LENCAfy = LENCA;
        RENCAfy = RENCA;
      }
      if (sensor.readRangeContinuousMillimeters() > 120) {
        Robot_Sequence = 7;
      }
      if (s == 1) {
        digitalWrite(RMotorEN_1, HIGH);
        digitalWrite(RMotorEN_2, HIGH);
        digitalWrite(LMotorEN_1, HIGH);
        digitalWrite(LMotorEN_2, HIGH);

        analogWrite(RMotorFWD, Rforward_SPEED);
        analogWrite(LMotorFWD, Lforward_SPEED);
      }
      if (s == 0) {
        digitalWrite(RMotorEN_1, HIGH);
        digitalWrite(RMotorEN_2, HIGH);
        digitalWrite(LMotorEN_1, HIGH);
        digitalWrite(LMotorEN_2, HIGH);

        analogWrite(RMotorFWD, 110);
        analogWrite(LMotorFWD, 80);
      }
      if (s == 2) {
        digitalWrite(RMotorEN_1, HIGH);
        digitalWrite(RMotorEN_2, HIGH);
        digitalWrite(LMotorEN_1, HIGH);
        digitalWrite(LMotorEN_2, HIGH);

        analogWrite(RMotorFWD, 80);
        analogWrite(LMotorFWD, 110);
      }
      break;
    case 8:
      get_MPU_data();
      angle = ypr[2] * 180 / M_PI;
      Serial.println(angle);
      if (angle >= 75) {
        digitalWrite(RMotorEN_1, LOW);
        digitalWrite(RMotorEN_2, LOW);
        digitalWrite(LMotorEN_1, LOW);
        digitalWrite(LMotorEN_2, LOW);

        analogWrite(RMotorFWD, 0);
        analogWrite(LMotorFWD, 0);
        analogWrite(RMotorBWD, 0);
        analogWrite(LMotorBWD, 0);
        statepreviousMillis = currentMillis;
        valueDetect = 1;
        visionState.data = valueDetect;
        pub.publish(&visionState);
        Robot_Sequence = 9;

      }
      if (angle < 75) {
        digitalWrite(RMotorEN_1, HIGH);
        digitalWrite(RMotorEN_2, HIGH);
        digitalWrite(LMotorEN_1, HIGH);
        digitalWrite(LMotorEN_2, HIGH);

        analogWrite(RMotorFWD, 150);
        analogWrite(LMotorFWD, 150);
      }
      break;

    case 9: //pausing, calibrating gyro reading for proceeding straight
      if (currentMillis - statepreviousMillis > 5000) {
        servoT.write(180);
        climb_RENCA = RENCA; // starts offset of RENCA - tracks how far robot is climbing up the beam
        climb_LENCA = LENCA; // starts offset of LENCA - tracks how far robot is climbing up the beam
        Robot_Sequence = 10;
      }
      break;

    case 10: //climbing sequence
      servoT.write(180);
      if (s == 6) { //centered
        digitalWrite(RMotorEN_1, HIGH);
        digitalWrite(RMotorEN_2, HIGH);
        digitalWrite(LMotorEN_1, HIGH);
        digitalWrite(LMotorEN_2, HIGH);
        analogWrite(RMotorFWD, 230);
        analogWrite(LMotorFWD, 242);
      }

      else if (s == 5) { // turn left while climbing
        adjust_LENCA = LENCA;
        Robot_Sequence = 99;
      }
      else if (s == 4) { //turn right while climbing
        adjust_RENCA = RENCA;
        Robot_Sequence = 100;
      }

      else { //drive "straight" while climbing
        digitalWrite(RMotorEN_1, HIGH);
        digitalWrite(RMotorEN_2, HIGH);
        digitalWrite(LMotorEN_1, HIGH);
        digitalWrite(LMotorEN_2, HIGH);
        analogWrite(RMotorFWD, 230);
        analogWrite(LMotorFWD, 242);
      }

      if ((LENCA - climb_LENCA) >= 12600 || (RENCA - climb_RENCA) >= 12600) { //ensures sensor will read after enough distance traveled
        vfilt = 1;
      }

      if (sensor.readRangeContinuousMillimeters() <= 120 && vfilt == 1) {
        
        Robot_Sequence = 11;
        climb_LENCA2 = LENCA;
        climb_RENCA2 = RENCA;
      }
      break;

    case 11: //descend the beam
      if (Robot_Sequence == 11) {
        get_MPU_data();
        angle = ypr[2] * 180 / M_PI;
        //Serial.println(angle);
      }
      if (((climb_LENCA2 - LENCA) >= 10000) || ((climb_RENCA2 - RENCA) >= 10000)) {
        statepreviousMillis = currentMillis;
        Robot_Sequence = 111;
      }

      if (rpm_R < -30) {
        turnLENCA = LENCA;
        digitalWrite(RMotorEN_1, HIGH);
        digitalWrite(RMotorEN_2, HIGH);
        analogWrite(RMotorFWD, Rdown_SPEED);
      }

      else {
        analogWrite(RMotorFWD, 0);
        digitalWrite(RMotorEN_1, HIGH);
        digitalWrite(RMotorEN_2, HIGH);
        analogWrite(RMotorBWD, 50);
      }
      if (rpm_L < -30) {  //-30 seemed better
        digitalWrite(LMotorEN_1, HIGH);
        digitalWrite(LMotorEN_2, HIGH);
        analogWrite(LMotorFWD, Ldown_SPEED);
      }
      else {
        analogWrite(LMotorFWD, 0);
        digitalWrite(LMotorEN_1, HIGH);
        digitalWrite(LMotorEN_2, HIGH);
        analogWrite(LMotorBWD, 50);
      }

      break;

    case 111:
      if (Robot_Sequence == 111) {
        get_MPU_data();
        angle = ypr[2] * 180 / M_PI;
        //Serial.println(angle);
      }
      if (angle < 10) {
        digitalWrite(RMotorEN_1, LOW);
        digitalWrite(RMotorEN_2, LOW);
        digitalWrite(LMotorEN_1, LOW);
        digitalWrite(LMotorEN_2, LOW);

        analogWrite(RMotorFWD, 0);
        analogWrite(LMotorFWD, 0);
        analogWrite(RMotorBWD, 0);
        analogWrite(LMotorBWD, 0);
        statepreviousMillis = currentMillis;
        Robot_Sequence = 12;
      }
      else if (angle > 10) {
        digitalWrite(RMotorEN_1, HIGH);
        digitalWrite(RMotorEN_2, HIGH);
        digitalWrite(LMotorEN_1, HIGH);
        digitalWrite(LMotorEN_2, HIGH);
        analogWrite(RMotorBWD, 255);
        analogWrite(LMotorBWD, 255);
        analogWrite(RMotorFWD, 0);
        analogWrite(LMotorFWD, 0);
      }
      break;

    case 99://Adjust Left while on beam
      if (LENCA - adjust_LENCA > 1200) {
        Robot_Sequence = 10;
      }
      else {
        digitalWrite(RMotorEN_1, HIGH);
        digitalWrite(RMotorEN_2, HIGH);
        digitalWrite(LMotorEN_1, HIGH);
        digitalWrite(LMotorEN_2, HIGH);

        analogWrite(RMotorFWD, 240);
        analogWrite(LMotorFWD, 223);
      }
      if ((LENCA - climb_LENCA) >= 12600 || (RENCA - climb_RENCA) >= 12600) { //ensures sensor will read after enough distance traveled
        vfilt = 1;
      }

      if (sensor.readRangeContinuousMillimeters() <= 120 && vfilt == 1) {
        Robot_Sequence = 11;
      }

      break;
    case 100://Adjust Right while on beam
      if (RENCA - adjust_RENCA > 1200) {
        Robot_Sequence = 10;
      }
      else {
        digitalWrite(RMotorEN_1, HIGH);
        digitalWrite(RMotorEN_2, HIGH);
        digitalWrite(LMotorEN_1, HIGH);
        digitalWrite(LMotorEN_2, HIGH);

        analogWrite(RMotorFWD, 223);
        analogWrite(LMotorFWD, 240);
      }
      if ((LENCA - climb_LENCA) >= 12600 || (RENCA - climb_RENCA) >= 12600) { //ensures sensor will read after enough distance traveled
        vfilt = 1;
      }

      if (sensor.readRangeContinuousMillimeters() <= 120 && vfilt == 1) {
        Robot_Sequence = 11;
      }
      break;
    case 12: // Wait to Push
      if ((currentMillis - statepreviousMillis > 2000)) {
        statepreviousMillis = currentMillis;
        Robot_Sequence = 13;
        //servoAngle = 45;
      }
      break;
    case 13://Push Off
      while (i < 50) {
        i++;
        servoAngle2 = map(i, 0, 50, 50, 0);
        servoL.write(i);
        servoR.write(servoAngle2);
        delay(10);
      }
      //delay(3000);
      if ((currentMillis - statepreviousMillis > 3000)) {
        servoL.write(50);
        servoR.write(0);
        Robot_Sequence = 14;
        Lbackward_SPEED = abs(Lforward_SPEED);
        Rbackward_SPEED = abs(Rforward_SPEED);

        int dir = 0;  // moving in y = 0 moving in x = 1
        LENCA = 0;  // reset to compare with stored encoder values from earlier
        RENCA = 0;
        // take negative of averages since LENCA and RENCA will increment in negative direction (reversing)
        xDist = ((LENCAx + RENCAx) / 2); // avg x distance of both encoders
        yDist = ((LENCAfy + RENCAfy) / 2) - ((LENCAsy + RENCAsy) / 2); //// avg y distance of both encoders

        //xDist = 4000;
        //yDist = 2000;
      }
      //Robot_Sequence = 14;
      break;

    // RETURN SEQUENCE CASE
    case 14://Move Y
      if ((abs(LENCA) < yDist) && (abs(RENCA) < yDist)) {
        digitalWrite(RMotorEN_1, HIGH);
        digitalWrite(RMotorEN_2, HIGH);
        digitalWrite(LMotorEN_1, HIGH);
        digitalWrite(LMotorEN_2, HIGH);
        analogWrite(RMotorBWD, Rbackward_SPEED);
        analogWrite(LMotorBWD, Lbackward_SPEED);
        analogWrite(RMotorFWD, 0);
        analogWrite(LMotorFWD, 0);
      }
      if ((abs(LENCA) >= yDist) && (abs(RENCA) >= yDist)) {
        Robot_Sequence = 15;
        LENCA = 0;  // reset to compare with stored encoder values from earlier
        RENCA = 0;
      }

      break;

    case 15:
      if (abs(RENCA) < 800) {
        digitalWrite(RMotorEN_1, HIGH);
        digitalWrite(RMotorEN_2, HIGH);
        digitalWrite(LMotorEN_1, HIGH);
        digitalWrite(LMotorEN_2, HIGH);

        analogWrite(RMotorBWD, 0);
        analogWrite(LMotorBWD, 80);
        analogWrite(RMotorFWD, 0);
        analogWrite(LMotorFWD, 0);
      }
      if (abs(RENCA) >= 800) {
        Robot_Sequence = 16;
        digitalWrite(RMotorEN_1, LOW);
        digitalWrite(RMotorEN_2, LOW);
        digitalWrite(LMotorEN_1, LOW);
        digitalWrite(LMotorEN_2, LOW);

        analogWrite(RMotorBWD, 0);
        analogWrite(LMotorBWD, 0);
        analogWrite(RMotorFWD, 0);
        analogWrite(LMotorFWD, 0);
        LENCA = 0;  // reset to compare with stored encoder values from earlier
        RENCA = 0;
      }
      break;

    case 16:
      if ((abs(LENCA) < xDist) && (abs(RENCA) < xDist)) {
        digitalWrite(RMotorEN_1, HIGH);
        digitalWrite(RMotorEN_2, HIGH);
        digitalWrite(LMotorEN_1, HIGH);
        digitalWrite(LMotorEN_2, HIGH);
        analogWrite(RMotorBWD, Rbackward_SPEED);
        analogWrite(LMotorBWD, Lbackward_SPEED);
        analogWrite(RMotorFWD, 0);
        analogWrite(LMotorFWD, 0);
      }
      if ((abs(LENCA) >= xDist) && (abs(RENCA) >= xDist)) {
        digitalWrite(RMotorEN_1, LOW);
        digitalWrite(RMotorEN_2, LOW);
        digitalWrite(LMotorEN_1, LOW);
        digitalWrite(LMotorEN_2, LOW);

        analogWrite(RMotorBWD, 0);
        analogWrite(LMotorBWD, 0);
        analogWrite(RMotorFWD, 0);
        analogWrite(LMotorFWD, 0);
        Robot_Sequence = 17;
      }
      break;
  }
}


void leftENCA() {

  if (digitalRead(LEncoderB) == HIGH) {

    LENCA++;
  }


  else {
    LENCA--;
  }
}


void rightENCA() {


  if (digitalRead(REncoderB) == HIGH) {

    RENCA--;
  }

  else {
    RENCA++;
  }
}

void get_MPU_data() { //Everything in this function has been written by the author of the MPU 6050 Library:


  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
}
