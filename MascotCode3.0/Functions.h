#include <math.h>
#include <EEPROM.h>
#include <AccelStepper.h>
#include <Servo.h>

// Include the libraries for the XBOX Controller
#include <XBOXONE.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

#include <Wire.h>

// Including the libraries for the IMU - acclerometer
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Include this library for matrix manipulation
#include <BasicLinearAlgebra.h>
// Library for writing and retreiving information using EEPROM
#include <EEPROM.h>

// Library for Nextion screen and GUI implementation
#include <SoftwareSerial.h>
#include <Nextion.h>

// Create State Machine
enum StateMachineState {
  MenuMode = 0,               // Press Start on X-BOX Controller
  ServoCalibration = 1,       // Press A on XBOX Controller
  Auto = 2,                   // Press R3 on XBOX Controller
  StepperHome = 3,            // Press Y on XBOX Controller
  StepperManual = 4,          // Press RB on XBOX Controller
  FindCoordinates = 5,        // Press RT on XBOX Controller
  SetCoordinates = 6,         // Press LT on XBOX Controller
  ServoManual = 7,            // Press LB on XBOX Controller
  NeckCalibration = 8,        // Press UP on XBOX Controller
  MoveToCalibrationState = 9, // Press DOWN on XBOX Controller
  Neck = 10,                  // Press LEFT on XBOX Controller
};
StateMachineState state = MenuMode;


// Initialize XBOX and USB Objects
USB Usb;
XBOXONE Xbox(&Usb);

// XBOX controller variables
int Analog_X = 0; //x-axis value
int Analog_Y = 0; //y-axis value
int Analog_R = 0; //r-axis value
int Analog_Z = 0;

int Analog_X_AVG = 0; //x-axis value average
int Analog_Y_AVG = 0; //y-axis value average
int Analog_R_AVG = 0; //r-axis value average

// More XBOX controller variables but these are float for improved precision.
float Analog_XN = 0; // Roll axis rotation
float Analog_YN = 0; // Pitch Axis rotation
float Analog_Yaw = 0; //Yah Servo rotation 

float Analog_XN_AVG = 0; //x-axis value average
float Analog_YN_AVG = 0; //y-axis value average
float Analog_RN_AVG = 0; //r-axis value average

/*
    Now initializing limit switch pins, stepper direction and pulse pins for shoulder and neck, servo pins
    Stepper objects and servo objects will also be defined
*/

int xEnd = 2; // X axis limit switch pin
int zEnd = 4; // Z axis limit switch pin
int yEnd = 7; // Y axis limit switch pin

// Declare Servo Pins - check wiring diagram if you have any questions
int xLPin = 11;
int zLPin = 5;
int xRPin = 6;
int zRPin = 3;
// Declaring neck servo pins
int NSPin = 12;

boolean limit = false;  // default value of limit switch reading

// NECK Stepper Direction and Pulse Pins
int onedir=40; 
int onepulse=42; 
int twodir=44; 
int twopulse=46; 
int threedir=48;  
int threepulse=14; //50 on PCB;

// SHOULDER Stepper Direction and Pulse Pins
int xdir = A1;
int xpulse = A0;
int ydir = A3;
int ypulse = A2;
int zdir = A5;
int zpulse = A4;

/* Initialize AccelStepper objects for each stepper motor.
   AccelStepper is an amazing library that offers a ton of great features for stepper control.

   Extra documentation and functions can be found here. https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
*/
AccelStepper stepperX(AccelStepper::DRIVER, xpulse, xdir); // X Axis Stepper
AccelStepper stepperY(AccelStepper::DRIVER, ypulse, ydir); // Y Axis Stepper
AccelStepper stepperZ(AccelStepper::DRIVER, zpulse, zdir); // Z Axis Stepper

// Initialize AccelStepper objects for each neck stepper motor.
AccelStepper stepperOne(AccelStepper::DRIVER, onepulse, onedir);        // front neck Stepper
AccelStepper stepperTwo(AccelStepper::DRIVER, twopulse, twodir);        // back right neck Stepper
AccelStepper stepperThree(AccelStepper::DRIVER, threepulse, threedir);  // back left neck Stepper

Servo NeckServo;  // Servo for Neck (YAW rotation)

Servo XservoL; // Left X Servo (Looks left and right)
Servo ZservoL; // Left Z Servo (Looks up and down)
Servo XservoR; // Right X Servo (Looks left and right)
Servo ZservoR; // Right Z Servo (Looks up and down)

/*
    Intializing robot variables
    And robot frame of references
*/
float eyeZlocation = 116.72;   // Distance between the center of the chin laser and center of the eye holders
float IPD = 69.5;   // Inter-pupillary Distance
/* Conversion factor for converting stepper motor steps to milimeters. This value was experimentally determined
    for each axis by making the axis move by a set amount of steps and then measuring the actual distance traveled.
    Taking the ratio between steps traveled and mm traveled leaves us with StepsPerMM.
*/
float StepsPerMM = 81; //was 80.22

/*
   Conversion factor of the angle of rotation of the servos in microseconds-per-degree.
*/
float microsecondsPerDegree = 10.20408163;
float leftMicrosecondsPerDegree = 10.20408163*0.98;
float rightMicrosecondsPerDegree = 10.20408163;
float leftXMicrosceondsPerDegree = 10.20408163*1.03;

/*
 * 90 degree position for the eye servos that is used in
 * parallax math. This is the value that is altered during servo calibration
 */
long int centerLeftXMicroseconds = 1500;
long int centerLeftZMicroseconds = 1500;
long int centerRightXMicroseconds = 1500;
long int centerRightZMicroseconds = 1500;

/* 
 * This variable is used in servo calibration to determine which motor is being calibrated 
 * 1 - right Z motor
 * 2 - left Z motor
 * 3 - right X motor
 * 4 - left X motor
 */
int calMotor = 0;

// Variables for xStick and zStick
float xStick = 0;
float zStick = 0;

float LrotationY = 0; //-M_PI/200
float RrotationY = 0; //1*(M_PI/200)

/* 
 * Initializing robot variables needed for the neck 
 */

float spoolRadius = 0.0071;               // radius of the spool that holds the thread
float spoolcircum = 2.0*M_PI*spoolRadius; // circumference of the spool
float mmperrevs = spoolcircum;            // mm of thread that each revolution of spool will loosen or tighten
float stepsperrev = 3838;                 // number of steps in one revolution for the neck steppers
float mmperstep = mmperrevs/stepsperrev;  // mm of thread released in one step

// Rotation variables
int pitch=0;
int roll=0;
int yaw=0;

float limN = 0.15;      // limiting value for determining if Neck stepper value needs to be updated.
float gainx = 0.1;
float gainy = 0.1;
float gainyaw = 0.1;

float desiredneckx = 0.0;
float desirednecky = 0.0;
float desiredneckyaw = 0.0;

float radiA = 0.06;                 // Distance from center of base plate to cable in meters
float radiB = 0.06;                 // Distance from center of top plate to cable in meters

/*
 * Length of the spring in meters. Gets updated after MoveToCalState()
 * It is 127 before calibration.
 */
float Lspring = 0.127; 
             
float PhiR = 0.0;                   //Amplitude of tilt
float PhiS = 0.0;                   //Direction of tilt

/*
 * Variables for the length of each of the cables
 * in the neck mechanism.
 */
float CableOneLength = 0.0;
float CableTwoLength = 0.0;
float CableThreeLength= 0.0;
/*
 * Variables for the speed of each of the steppers in the neck.
 * Currently unused but will be applied to control velocity of steppers.
 */
float VelocityStepperOne = 0.0;
float VelocityStepperTwo=0.0;
float VelocityStepperThree = 0.0;

/*
 * Neck stepper variables that are updated during neck calibration.
 */
float CalibrationStepperOne = 0.0;
float CalibrationStepperTwo = 0.0;
float CalibrationStepperThree = 0.0;

/*
 * Neck cable length variables that are updated during neck calibration.
 */
float CalibrationCableLengthOne = CalibrationStepperOne*mmperstep;
float CalibrationCableLengthTwo = CalibrationStepperTwo*mmperstep;
float CalibrationCableLengthThree = CalibrationStepperThree*mmperstep;

/*
 * Variables with the name last before their name, store values of
 * the last neck mechanism configuration. This is necessary for 
 * the neck to revert 
 */
float laststepperposOne = 0.0;
float laststepperposTwo = 0.0;
float laststepperposThree = 0.0;

float lastCableLengthOne = laststepperposOne*mmperstep;
float lastCableLengthTwo = laststepperposTwo*mmperstep;
float lastCableLengthThree = laststepperposThree*mmperstep;

float lastservopos = 0.0;

// Use these in set up (first time) and always for calibration
float knownCableLengthOne = 0.127;
float knownstepperposOne = knownCableLengthOne/mmperstep;
float knownCableLengthTwo = 0.127; 
float knownstepperposTwo = knownCableLengthTwo/mmperstep;
float knownCableLengthThree = 0.127;
float knownstepperposThree = knownCableLengthThree/mmperstep;

// Variables needed to calculate move
float desiredstepperposOne = 0.0; 
float desiredstepperposTwo = 0.0; 
float desiredstepperposThree = 0.0; 

float deltadistanceOne = 0.0;
float deltadistanceTwo = 0.0;
float deltadistanceThree = 0.0;

/*
   Now we are going to be defining the robot's frame of references

   S - will be the screen frame of reference with the zero coordinate at the center of the screen,
    X will be to the right, Z up, and Y into the screen
   B - will be the origin frame of reference with the zero coordinate at the position where the
    stages of the 3-axis shoulder steppers hit the limit switches or zero position
   C - will be the origin frame of reference located at the center of the Z-axis stage,
    that the neck and eye mechanisms are mounted to.
    This point also closely coincides with the center of the chin laser
   D - will be the frame of reference at the center of the eye mechanisms,
    with Z up, X to the right and Y along the axis of the straight 90-degree eyes
   L - will be the left eye's frame of reference with the origin at the center of rotation
    and is aligned with the D frame when the 90-degree angle is commanded
   R - will be the right eye's frame of reference with the origin at the center of rotation
    and is aligned with D frame when the 90-degree angle is commanded
*/

/*
   Functions to create a transformation matrix using the X,Y,Z
   distances between frames of references.
*/

BLA::Matrix<3,3> eye()
{
  BLA::Matrix<3,3> I = {1, 0, 0,
                        0, 1, 0,
                        0, 0, 1};
  return I;
}
 
 
BLA::Matrix<3,3> hat(BLA::Matrix<3> v)
{
  BLA::Matrix<3,3> vhat;
  vhat.Fill(0);
  vhat(0,1) = -v(2);
  vhat(0,2) = v(1);
  vhat(1,0) = v(2);
  vhat(1,2) = -v(0);
  vhat(2,0) = -v(1);
  vhat(2,1) = v(0);
  
  return vhat;
}
 
float norm(BLA::Matrix<3> v)
{
    return sqrt( v(0)*v(0)+ v(1)*v(1) + v(2)*v(2)); 
}

float norm_6(BLA::Matrix<6> v)
{
  return sqrt( v(0)*v(0)+ v(1)*v(1) + v(2)*v(2)+ v(3)*v(3)+ v(4)*v(4)+ v(5)*v(5));
}
 
BLA::Matrix<3,3> rodrigues(BLA::Matrix<3> v)
{
    float theta = norm(v);

    if (theta == 0.0)
    {
      return eye();
    }
    else
    {
      BLA::Matrix<3> vnorm = v/norm(v);
    
 
      BLA::Matrix<3,3> vhat = hat(vnorm);
  
      BLA::Matrix<3,3> R = eye() + vhat*sin(theta) + vhat*vhat*(1-cos(theta));
  
      return R;
    }
}

BLA::Matrix<4,4> expmxi(BLA::Matrix<6> xi)
{   
  
  BLA::Matrix<3> rotvec = {xi(3),xi(4),xi(5)};
  BLA::Matrix<3> tvec = {xi(0), xi(1), xi(2)};
  
  BLA::Matrix<3,3> R = rodrigues(rotvec);
  
  BLA::Matrix<4,4> g = {R(0,0), R(0,1), R(0,2), tvec(0),
                        R(1,0), R(1,1), R(1,2), tvec(1),
                        R(2,0), R(2,1), R(2,2), tvec(2),
                              0,      0,      0,       1};
  return g;
}

BLA::Matrix<3,3> OuterProduct (BLA::Matrix<3> w)
{
  BLA::Matrix<3,3> product;
  
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      product(i,j) = w(i) * w(j);
    }
  }

  return product;
}

BLA::Matrix<3> CrossProduct (BLA::Matrix<3> w, BLA::Matrix<3> v)
{
  BLA::Matrix<3,3> w_hat = hat(w);
  return w_hat*v;
}

BLA::Matrix<4,4> expmxi_Correct(BLA::Matrix<6> xi)
{
  BLA::Matrix<3> w_noNorm = {xi(3), xi(4), xi(5)};
  float theta = norm(w_noNorm);
  BLA::Matrix<6> xiNorm = xi;

  if (theta != 0)
  {
    xiNorm = xi/theta;
  }

  BLA::Matrix<3> v = {xiNorm(0), xiNorm(1), xiNorm(2)};
  BLA::Matrix<3> w = {xiNorm(3), xiNorm(4), xiNorm(5)};

  BLA::Matrix<3,3> R = rodrigues(w*theta);

  BLA::Matrix<3,3> I = eye();

  BLA::Matrix<3,3> outProduct = OuterProduct(w);
  BLA::Matrix<3> crossProduct = CrossProduct(w,v);

  BLA::Matrix<3> T = ((I - R) * crossProduct) + (outProduct * v * theta);

  BLA::Matrix<4,4> g = {R(0,0), R(0,1), R(0,2), T(0),
                        R(1,0), R(1,1), R(1,2), T(1),
                        R(2,0), R(2,1), R(2,2), T(2),
                              0,      0,      0,       1};

  return g;
}

BLA::Matrix<3> EVector(BLA::Matrix<4,4> gg03)
{
  float alpha = (atan2(gg03(1,0),gg03(0,0)))*(180.00/M_PI);
  float beta = (atan2(-gg03(2,0),(sqrt(sq(gg03(2,1))+sq(gg03(2,2))))))*(180.00/M_PI);
  float gamma = (atan2(gg03(2,1),gg03(2,2)))*(180.00/M_PI);
  BLA::Matrix<3> EulerVector = {alpha, beta, gamma};

  return EulerVector;
}

BLA::Matrix<3> TVector(BLA::Matrix<4,4> gg03)
{
  BLA::Matrix<3> TranslationV = {gg03(0,3), gg03(1,3), gg03(2,3)};
  return TranslationV;
}

BLA::Matrix<4, 4> xform(float alpha, float beta, float gamma, float x, float y, float z) {
  
  BLA::Matrix<3> rotvec = {alpha,beta, gamma};
  BLA::Matrix<3> tvec = {x, y, z};
  
  BLA::Matrix<3,3> R = rodrigues(rotvec);
  
  BLA::Matrix<4,4> g = {R(0,0), R(0,1), R(0,2), tvec(0),
                        R(1,0), R(1,1), R(1,2), tvec(1),
                        R(2,0), R(2,1), R(2,2), tvec(2),
                              0,      0,      0,       1};
  return g;
}

void PrintG(BLA::Matrix<4,4> g)
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      float x = g(i,j);
      Serial.print(x,6);
      Serial.print("     ");
    }
    Serial.println(" ");
  }
}

/*
  This is the distance between our origin coordinate system B
  (when steppers hit the limit switches
  + offsets to the center of the stage) to the origin of the screen S.
*/
BLA::Matrix<4, 4> gBS = xform(0,0,0,(141.23), 562, (231.23)); //140.15, 220.46

/*
   This is the distance between our B coordinate system to the stage connected to X,Y,Z steppers
   (C coordinate system). We are going to be filling it with zeros and then initializing
   this variable during run-time as it requires real-time input
*/
BLA::Matrix<4, 4> gBC = xform(0,0,0,-stepperX.currentPosition() / StepsPerMM,
                              -stepperY.currentPosition() / StepsPerMM,
                              stepperZ.currentPosition() / StepsPerMM);

/*
   (stage connected to X,Y,Z steppers) to the imaginary point
   in between the eyes(D coordinate system).
*/
BLA::Matrix<4, 4> gCD = xform(0,0,0,0, 175, 290);

/*
   This is the distance between our D coordinate system to the center of the left eye L.
*/
BLA::Matrix<4, 4> gLD = xform(0,LrotationY,0,(-34.75), 0, 0);

/*
   This is the distance between our D coordinate system to the center of the right eye R.
*/
BLA::Matrix<4, 4> gRD = xform(0,RrotationY,0,(34.75), 0, 0);


/*
   Now we will calculate the frame transformation from the left and right eyes to the screen
   Transforming the frame will allow to know what the desired position on the screen is
   with respect to the frame of reference of the eyes.
*/
BLA::Matrix<4, 4> gLS = gLD.Inverse() * gCD.Inverse() * gBC.Inverse() * gBS;

BLA::Matrix<4, 4> gRS = gRD.Inverse() * gCD.Inverse() * gBC.Inverse() * gBS;

/*
   Now we will declare 3 Dot position coordinates matrices of gaze location in the screen's frame of reference,
   the left eye's frame of reference and the right eye's frame of reference.
*/

int calPointDotPos = 0;

/*
 * Calibration points for the Tobii eye calibration routine
 */
BLA::Matrix<4, 4> calPoints = {0, 0, 0, 1,
                               104.57, 0, -68.79, 1,
                               0, 0, 69.66, 1,
                               -104.57, 0, -68.79, 1};

BLA::Matrix<4> screenDotPos = {0, 0, 0, 1};
BLA::Matrix<4> leftDotPos = gLS * screenDotPos;
BLA::Matrix<4> rightDotPos = gRS * screenDotPos;


/*
   Declaring rotation angles for the left and right servos.
   These rotation angles will be used in the parallax function to rotate the servos
   to make the eyes converge
*/
float alphaLeft = 0;    // alpha is the rotation in the X axis.
float alphaRight = 0;
float betaLeft = 0;     // beta is the rotation in the Z axis.
float betaRight = 0;

float alphaLeftPrev = 0;
float betaLeftPrev = 0;

#include "StepperHome.h"
#include "ServoManual.h"
#include "StepperManual.h"
//#include "GUI.h"
#include "NeckStepper.h"

/*
 * Function to update the transformation matrices
 */
void UpdateTransformation()
{
  float yawAngle = (((NeckServo.read())-83)*2) * (3.14159/180);
  // Recalculating gBC and gLS, gRS
  gBC = xform(0,0,0,-stepperX.currentPosition() / StepsPerMM,
                              -stepperY.currentPosition() / StepsPerMM,
                              stepperZ.currentPosition() / StepsPerMM);
                              
  gCD = xform(0,0,yawAngle,0, 175, 290);
  gLS = gLD.Inverse() * gCD.Inverse() * gBC.Inverse() * gBS;

  gRS = gRD.Inverse() * gCD.Inverse() * gBC.Inverse() * gBS;
  
  Serial.println("Transformation Updated");
}

/*
 * Function to write the calibration variables for SHOULDER steppers
 * and EYE SERVOS to EEPROM
 */
void WriteCalibrationVariablesToProm()
{
  int eeAddress = 0;

  EEPROM.put(eeAddress, stepperX.currentPosition());
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, stepperY.currentPosition());
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, stepperZ.currentPosition());
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, centerLeftXMicroseconds);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, centerLeftZMicroseconds);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, centerRightXMicroseconds);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, centerRightZMicroseconds);

  delay(100);

  Serial.println("Variables loaded!");
}

/*
 * Function to read the calibration variables for SHOULDER steppers
 * and eye servos from EEPROM
 */
void ReadCalibrationVariablesFromProm()
{
  int eeAddress = 0;

  long int stepperPosition = 0;
  long int servoCenter = 0.0;
  
  EEPROM.get(eeAddress, stepperPosition);
  stepperX.setCurrentPosition(stepperPosition);
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, stepperPosition);
  stepperY.setCurrentPosition(stepperPosition);
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, stepperPosition);
  stepperZ.setCurrentPosition(stepperPosition);

  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, servoCenter);
  centerLeftXMicroseconds = servoCenter;
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, servoCenter);
  centerLeftZMicroseconds = servoCenter;
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, servoCenter);
  centerRightXMicroseconds = servoCenter;
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, servoCenter);
  centerRightZMicroseconds = servoCenter;

  Serial.println(centerLeftXMicroseconds);
  Serial.println(centerLeftZMicroseconds);
  Serial.println(stepperX.currentPosition());
  Serial.println(stepperY.currentPosition());
  
  Serial.println("Variables read!");
  UpdateTransformation();
}

/*
 * Function to write the last NECK stepper position to EEPROM
 */
void WriteLastStepperPosToProm()
{
  int eeAddress = 28;

  EEPROM.put(eeAddress, stepperOne.currentPosition());
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, stepperTwo.currentPosition());
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, stepperThree.currentPosition());
  eeAddress += sizeof(float);
  delay(100);

  Serial.println("Variables loaded!");
}

/*
 * Function to write the calibration NECK stepper positions to EEPROM 
 */
void WriteCalibrationStepperPosToProm()
{
  int eeAddress = 40;

  EEPROM.put(eeAddress, stepperOne.currentPosition());
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, stepperTwo.currentPosition());
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, stepperThree.currentPosition());
  eeAddress += sizeof(float);
  delay(100);
  //EEPROM.put(eeAddress, NeckServo.read());
  //eeAddress += sizeof(float);
  Serial.println("Variables loaded!");
}

/*
 * Function to read the last NECK stepper position from EEPROM
 */
void ReadLastStepperPosFromProm()
{
  int eeAddress = 28;

  long int stepperPosition = 0;
  
  EEPROM.get(eeAddress, stepperPosition);
  laststepperposOne = stepperPosition;
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, stepperPosition);
  laststepperposTwo = stepperPosition;
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, stepperPosition);
  laststepperposThree = stepperPosition;
  
  Serial.println("Variables read!");
}

/*
 * Function to read the calibration NECK stepper positions from EEPROM
 */
void ReadCalibrationStepperPosFromProm()
{
  int eeAddress = 40;

  long int stepperPosition = 0;
  
  EEPROM.get(eeAddress, stepperPosition);
  CalibrationStepperOne = stepperPosition;
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, stepperPosition);
  CalibrationStepperTwo = stepperPosition;
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, stepperPosition);
  CalibrationStepperThree = stepperPosition;
  
  Serial.println("Variables read!");
}

/*
 * Function to print out the neck angles - pitch, roll and yaw
 * to serial monitor.
 */
void NeckAngles()
{
  //Tell the sensor we want to pull data
  sensors_event_t event; 
  bno.getEvent(&event);
  
  //Pull angles
  yaw = event.orientation.x; //yaw
  roll = event.orientation.y; //roll
  pitch = event.orientation.z; //pitch
  
  //Write to Serial Monitor 
  Serial.print("Roll: ");
  Serial.println(roll);
  Serial.print("Pitch: ");
  Serial.println(pitch);
  Serial.print("Yah: ");
  Serial.println(yaw);
}

/*
 * 
 */
void EulerVectorI()
{    
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  Serial.print("Alpha: ");
  Serial.print(euler.x());
  Serial.print(" Beta: ");
  Serial.print(euler.y());
  Serial.print(" Gamma: ");
  Serial.print(euler.z());
  Serial.print("\t\t");
}

/*
   Function to check if a XBOX button has been pressed.
*/
bool IfButtonPressed()
{
  bool pressed = false;

  //Conditional statements to change the state if desired.
  if (Xbox.getButtonClick(START))
  {
    state = MenuMode;
    pressed = true;
  }
  if (Xbox.getButtonClick(Y))
  {
    state = StepperHome;
    pressed = true;
  }
  if (Xbox.getButtonClick(A))
  {
    state = ServoCalibration;
    pressed = true;
  }
  if (Xbox.getButtonClick(R3))
  {
    state = Auto;
    pressed = true;
  }
  if (Xbox.getButtonClick(R2))
  {
    state = FindCoordinates;
    pressed = true;
  }
  if (Xbox.getButtonClick(L2))
  {
    state = SetCoordinates;
    pressed = true;
  }
  if (Xbox.getButtonClick(R1))
  {
    state = StepperManual;
    pressed = true;
  }
  if (Xbox.getButtonClick(L1))
  {
    state = ServoManual;
    pressed = true;
  }  
  if (Xbox.getButtonClick(UP))
  {
    state = NeckCalibration;
    pressed = true;
  }
  if (Xbox.getButtonClick(DOWN))
  {
    state = MoveToCalibrationState;
    pressed = true;
  }
  if (Xbox.getButtonClick(LEFT))
  {
    state = Neck;
    pressed = true;
  }
  return pressed;
}

/*
   Menu Mode state that waits for the XBOX button to be pressed
   Switches state to the state corresponding to that button
*/
void runMenuModeState()
{
  IfButtonPressed();
}

/*
   Servo Calibration State that will be used to calibrate the servos of the eyes
   Calibration sequence will include pointing the eye lasers to the 90-degree position of the servos
   on the screen.
*/
void runServoCalibrationState()
{
  int i = 0;

  while (i == 0)
  {
    servoCalibration();
    parallax();

    Usb.Task();
    if (Xbox.getButtonClick(B))
    {
      WriteCalibrationVariablesToProm();
      i = 1;
      state = MenuMode;
    }
  }
}

/*
   Servo manual state will be used to control the eye servos manually using the XBOX controller.
*/
void runServoManualState()
{
//  for (int i = 83; i < 100; i+=20)
//  {
//    NeckServo.write(i);
//    UpdateTransformation();
//    GetScreenDotPosition();
//    parallax();
//    delay(1000);
//  }
//  
//  for (int i = 100; i > 83; i-=20)
//  {
//    NeckServo.write(i);
//    UpdateTransformation();
//    GetScreenDotPosition();
//    parallax();
//    delay(1000);
//  }
//  for (int i = 83; i > 65; i-=20)
//  {
//    NeckServo.write(i);
//    UpdateTransformation();
//    GetScreenDotPosition();
//    parallax();
//    delay(1000);
//  }
//  for (int i = 65; i < 83; i+=20)
//  {
//    NeckServo.write(i);
//    UpdateTransformation();
//    GetScreenDotPosition();
//    parallax();
//    delay(1000);
//  }
   NeckServo.write(100);
   UpdateTransformation();
   GetScreenDotPosition();
   parallax();
  

  IfButtonPressed();
}

/*
   Stepper Home state will home the steppers. Homing the steppers will include moving the
   steppers till they hit the limit switches. Hitting the limit switches will reset the zero locations
   of the steppers.
*/
void runStepperHomeState()
{
  zeroStepper(ydir, ypulse, yEnd);
  stepperY.setCurrentPosition(500);
  zeroStepper(xdir, xpulse, xEnd); // DO NOT switch with Z
  stepperX.setCurrentPosition(-500);
  zeroStepper(zdir, zpulse, zEnd); // DO NOT switch with X
  stepperZ.setCurrentPosition(500);

  UpdateTransformation();
  state = MenuMode;
}

/*
   Manual Stepper state will enables us to control the stepper positions
   using the XBOX controller.
*/
void runStepperManualState()
{
  while (IfButtonPressed() == false)
  {
    Usb.Task();       // Requesting data from the XBOX controller.
    ReadAnalog();     // Take the Xbox controller data and convert to desired stepper axis movements
    stepperX.run();   // Run function in accelstepper actually moves the steppers to the location
    stepperY.run();   // determined by .move() function called in ReadAnalog.
    stepperZ.run();
  }
  WriteCalibrationVariablesToProm();

  UpdateTransformation();
}

/*
 * Function that will move the eyes of the robot
 * to the Tobii Calibration Coordinates.
 */
void runSetCoordinatesState()
{
  int i = 0;
  while (i < 4)
  {
    for (int j = 0; j < 3; j++)
    {
      screenDotPos(j) = calPoints(i,j);
    }
    leftDotPos = gLS * screenDotPos;
    rightDotPos = gRS * screenDotPos;
    parallax();

    Usb.Task();
    if (Xbox.getButtonClick(B))
    {
      i++;
    }
    else
    {
      continue;
    }
  }
  state = MenuMode;
}

/*
 * Function that will print out the coordinates of the gaze
 * of the eyes on the Serial Monitor.
 */
void runFindCoordinatesState()
{
  GetScreenDotPosition();
  parallax();

  Usb.Task();
  if (Xbox.getButtonClick(B))
  {
    Serial << screenDotPos;
    Serial.println("\n");
  }
  IfButtonPressed();
}

/*
 * Function that will be used to calibrate the neck steppers
 * to their 0 angle position.
 */
void runNeckCalibrationState()
{
  Serial.println("I am here 1");
  Usb.Task();
  
  int i = 0;
  int j = 0;
  int k = 0;

  /*
   * When the X button is pressed, write the calibration variables to prom
   * write the last neck stepper configuration to prom
   * get out of neck calibration state
   */
  if(Xbox.getButtonClick(X))
  {
    Serial.println("I am here 1");
    WriteCalibrationStepperPosToProm();
    WriteLastStepperPosToProm();
    state = MenuMode; 
  }
  /*
   * When the Y button is pressed, we start calibrating neck steppers one by one
   * Neck Stepper One is the front stepper of the neck.
   * Neck Stepper Two is the back right stepper.
   * Neck Stepper Three is the back left stepper.
   */
  else if(Xbox.getButtonClick(Y))
  {
    Usb.Task();
    Serial.println("I am here 2");

    // Stepper One
    while (i == 0)
    {
      Usb.Task();
      
      if (Xbox.getButtonClick(UP))
      {
        stepperOne.runToNewPosition(knownstepperposOne+100); //loosen
        stepperOne.setCurrentPosition(knownstepperposOne); 
        stepperOne.setMaxSpeed(3000);                        //SPEED = Steps / second 
        stepperOne.setAcceleration(3000);                    //ACCELERATION = Steps /(second)^2
        NeckAngles();  
      }
      else if (Xbox.getButtonClick(DOWN))
      {
        stepperOne.runToNewPosition(knownstepperposOne-100); //tightening 
        stepperOne.setCurrentPosition(knownstepperposOne); 
        stepperOne.setMaxSpeed(3000);                       //SPEED = Steps / second 
        stepperOne.setAcceleration(3000);                   //ACCELERATION = Steps /(second)^2    
        NeckAngles();
      }
      else if (Xbox.getButtonClick(B))
      {
        i = 1;
      }
    }

    // Stepper Two
    while (j == 0)
    {
       Usb.Task();
       if (Xbox.getButtonClick(UP))
      {
        stepperTwo.runToNewPosition(knownstepperposTwo+100); //loosen
        stepperTwo.setCurrentPosition(knownstepperposTwo); 
        stepperTwo.setMaxSpeed(3000);                       //SPEED = Steps / second 
        stepperTwo.setAcceleration(3000);                   //ACCELERATION = Steps /(second)^2
        NeckAngles();  
      }
      else if (Xbox.getButtonClick(DOWN))
      {
        stepperTwo.runToNewPosition(knownstepperposTwo-100); //tightening 
        stepperTwo.setCurrentPosition(knownstepperposTwo); 
        stepperTwo.setMaxSpeed(3000);                       //SPEED = Steps / second 
        stepperTwo.setAcceleration(3000);                   //ACCELERATION = Steps /(second)^2    
        NeckAngles();
      }
      else if (Xbox.getButtonClick(B))
      {
        j = 1;
      }
    }

    // Stepper Three
    while (k == 0)
    {
      Usb.Task();
      if (Xbox.getButtonClick(UP))
        {
          stepperThree.runToNewPosition(knownstepperposThree+100); //loosen
          stepperThree.setCurrentPosition(knownstepperposThree); 
          stepperThree.setMaxSpeed(3000);                         //SPEED = Steps / second 
          stepperThree.setAcceleration(3000);                     //ACCELERATION = Steps /(second)^2
          NeckAngles();  
        }
        else if (Xbox.getButtonClick(DOWN))
        {
          stepperThree.runToNewPosition(knownstepperposThree-100); //tightening 
          stepperThree.setCurrentPosition(knownstepperposThree); 
          stepperThree.setMaxSpeed(3000);                         //SPEED = Steps / second 
          stepperThree.setAcceleration(3000);                     //ACCELERATION = Steps /(second)^2    
          NeckAngles();
        }
        else if (Xbox.getButtonClick(B))
        {
          k = 1; 
        } 
    }
  }
}

/*
 * Function to move the neck steppers to their calibrated position.
 */
void runMoveToCalibrationState()
{
  
  stepperOne.runToNewPosition(CalibrationStepperOne);   //loosen
  stepperOne.setCurrentPosition(CalibrationStepperOne); 
  stepperOne.setMaxSpeed(3000);                         //SPEED = Steps / second 
  stepperOne.setAcceleration(3000);                     //ACCELERATION = Steps /(second)^2

  stepperTwo.runToNewPosition(CalibrationStepperTwo); //loosen
  stepperTwo.setCurrentPosition(CalibrationStepperTwo); 
  stepperTwo.setMaxSpeed(3000);                       //SPEED = Steps / second 
  stepperTwo.setAcceleration(3000);                   //ACCELERATION = Steps /(second)^2

  stepperThree.runToNewPosition(CalibrationStepperThree); //loosen
  stepperThree.setCurrentPosition(CalibrationStepperThree); 
  stepperThree.setMaxSpeed(3000);                         //SPEED = Steps / second 
  stepperThree.setAcceleration(3000);                     //ACCELERATION = Steps /(second)^2

  EulerVectorI();
  NeckAngles();
  Lspring = ((CalibrationStepperOne*mmperstep)+(CalibrationStepperTwo*mmperstep)+(CalibrationStepperThree*mmperstep))/3.0;
  Serial.println(Lspring*1000);
  WriteLastStepperPosToProm();
  state = MenuMode;  
}
    
/*
 * Function to obtain analog control of the neck.
 */
void runNeckState()
{
   while (IfButtonPressed() == false)
    {
      Usb.Task();         // Request data from Xbox controller
      ReadAnalogNeck();   // Take Xbox controller data and convert to desired stepper axis movement
      stepperOne.run(); 
      stepperTwo.run();
      stepperThree.run();
      //NeckAngles();
    }
    WriteLastStepperPosToProm();
    NeckAngles();
    EulerVectorI();
    Serial.println("Am here");
}

/*
 * Function to obtain the average calibration variables for the XBOX analog sticks.
 */
void InitialValues()
{
  //Set the values to zero before averaging
  float tempX = 0;
  float tempY = 0;
  float tempR = 0;
  float tempXN = 0;
  float tempYN = 0;
  float tempRN = 0;
  //----------------------------------------------------------------------------
  //read the analog 50x, then calculate an average.
  //they will be the reference values
  for (int i = 0; i < 50; i++)
  {
    Usb.Task();
    //     tempX += analogRead(Analog_X_pin);
    tempX += map(Xbox.getAnalogHat(LeftHatX), -32767, 32767, 0, 1023);
    tempXN += mapf(Xbox.getAnalogHat(LeftHatX), -32767, 32767, -1, 1);
    delay(10); //allowing a little time between two readings
    //     tempY += analogRead(Analog_Y_pin);
    tempY += map(Xbox.getAnalogHat(LeftHatY), -32767, 32767, 0, 1023);
    tempYN += mapf(Xbox.getAnalogHat(LeftHatY), -32767, 32767, -1, 1);
    delay(10);
    tempR += map(Xbox.getAnalogHat(RightHatX), -32767, 32767, 0, 1023);
    tempRN += mapf(Xbox.getAnalogHat(RightHatX), -32767, 32767, -1, 1);
    delay(10);
  }
  //----------------------------------------------------------------------------
  Analog_X_AVG = tempX / 50;
  Analog_Y_AVG = tempY / 50;
  Analog_R_AVG = tempR / 50;
  Analog_XN_AVG = tempXN / 50;
  Analog_YN_AVG = tempYN / 50;
  Analog_RN_AVG = tempRN / 50;
  //----------------------------------------------------------------------------
  Serial.print("AVG_X: ");
  Serial.println(Analog_X_AVG);
  Serial.print("AVG_Y: ");
  Serial.println(Analog_Y_AVG);
  Serial.print("AVG_R: ");
  Serial.println(Analog_R_AVG);
  Serial.println("Calibration finished");
}

float autoCoordinates[2][294] = {{69.21,
52.27,
-37.24,
-56.47,
7.15,
52.27,
-70.3,
7.15,
-50.49,
52.27,
-37.24,
-37.24,
52.27,
-35.14,
33.38,
69.21,
-10.75,
82,
7.15,
34.51,
70.08,
-10.75,
7.15,
35.4,
33.38,
-70.3,
-10.75,
7.15,
-37.24,
70.08,
-28.86,
7.15,
48.87,
-56.47,
7.15,
70.08,
-28.86,
7.15,
70.08,
-10.75,
16.35,
-56.47,
-28.86,
7.15,
-87.28,
69.21,
-70.3,
7.15,
95.97,
-87.28,
-10.75,
52.27,
-56.47,
34.51,
-10.75,
-70.3,
82,
7.15,
69.21,
52.27,
-37.24,
-56.47,
7.15,
52.27,
-70.3,
7.15,
-50.49,
52.27,
-37.24,
-37.24,
52.27,
-35.14,
33.38,
69.21,
-10.75,
7.15,
-37.24,
70.08,
-28.86,
7.15,
-56.47,
-16.82,
-56.47,
-28.86,
14.68,
70.08,
34.51,
-56.47,
82,
7.15,
-37.24,
52.27,
34.51,
-50.49,
52.27,
34.51,
-16.86,
7.15,
-70.25,
-87.28,
14.68,
-70.3,
7.15,
-10.75,
70.08,
7.15,
48.87,
-87.28,
52.67,
-56.47,
7.15,
69.21,
52.27,
-37.24,
-56.47,
7.15,
48.87,
-56.47,
-87.28,
34.51,
52.27,
34.51,
-16.86,
-37.24,
33.38,
69.21,
7.15,
-87.28,
34.51,
-50.49,
7.15,
95.97,
33.38,
-28.86,
95.97,
70.08,
-70.3,
-56.47,
-37.24,
33.38,
69.21,
7.15,
-87.28,
34.51,
-50.49,
7.15,
-28.86,
-56.47,
-70.25,
-87.28,
-28.86,
-50.49,
52.27,
34.51,
-16.86,
67.73,
7.15,
-50.49,
70.08,
52.27,
34.51,
-16.86,
7.15,
-10.75,
16.35,
-56.47,
7.15,
-87.28,
-35.14,
-10.75,
52.27,
-16.82,
52.27,
-10.75,
52.27,
-56.47,
-70.3,
7.15,
-10.75,
16.35,
-87.28,
-10.75,
7.15,
14.68,
70.08,
33.38,
7.15,
69.21,
70.08,
-16.82,
-56.47,
7.15,
-87.28,
34.51,
-50.49,
7.15,
-70.3,
95.97,
-56.47,
34.51,
-50.49,
52.27,
34.51,
-16.86,
7.15,
-10.75,
52.27,
48.87,
-56.47,
7.15,
-70.25,
52.27,
-10.75,
16.35,
7.15,
-10.75,
16.35,
-56.47,
7.15,
95.97,
-56.47,
70.08,
95.97,
69.21,
-56.47,
7.15,
-10.75,
16.35,
-87.28,
-10.75,
7.15,
14.68,
70.08,
33.38,
7.15,
69.21,
70.08,
-16.82,
-56.47,
67.73,
7.15,
52.27,
7.15,
-10.75,
16.35,
52.27,
34.51,
52.67,
7.15,
-10.75,
16.35,
-87.28,
-10.75,
7.15,
52.27,
-70.3,
7.15,
-10.75,
16.35,
-56.47,
7.15,
48.87,
-56.47,
-87.28,
34.51,
52.27,
34.51,
-16.86,
7.15,
70.08,
-37.24,
7.15,
-10.75,
16.35,
52.27,
-70.3,
7.15,
16.35,
33.38,
48.87,
-87.28,
34.51,
7.15,
-56.47,
-46.43,
95.97,
-56.47,
-28.86,
52.27,
-56.47,
34.51,
-35.14,
-56.47,
82
},
                                  {-60.13,
-33.44,
-51.55,
-33.44,
-5.32,
-33.44,
-59.94,
-5.32,
-59.94,
-33.44,
-51.55,
-51.55,
-33.44,
-69.02,
-33.44,
-60.13,
-33.44,
-56.9,
-5.32,
-75.77,
-33.44,
-33.44,
-5.32,
-59.2,
-33.44,
-59.94,
-33.44,
-5.32,
-51.55,
-33.44,
-33.44,
-5.32,
-80.47,
-33.44,
-5.32,
-33.44,
-33.44,
-5.32,
-33.44,
-33.44,
-57.76,
-33.44,
-33.44,
-5.32,
-59.94,
-60.13,
-59.94,
-5.32,
-33.44,
-59.94,
-33.44,
-33.44,
-33.44,
-75.77,
-33.44,
-59.94,
-56.9,
-5.32,
-60.13,
-33.44,
-51.55,
-33.44,
-5.32,
-33.44,
-59.94,
-5.32,
-59.94,
-33.44,
-51.55,
-51.55,
-33.44,
-69.02,
-33.44,
-60.13,
-33.44,
-5.32,
-51.55,
-33.44,
-33.44,
-5.32,
-33.44,
-73.35,
-33.44,
-33.44,
-33.44,
-33.44,
-75.77,
-33.44,
-56.9,
-5.32,
-51.55,
-33.44,
-75.77,
-59.94,
-33.44,
-75.77,
-54.72,
-5.32,
-33.44,
-59.94,
-33.44,
-59.94,
-5.32,
-33.44,
-33.44,
-5.32,
-80.47,
-59.94,
-61.4,
-33.44,
-5.32,
-60.13,
-33.44,
-51.55,
-33.44,
-5.32,
-80.47,
-33.44,
-59.94,
-75.77,
-33.44,
-75.77,
-54.72,
-51.55,
-33.44,
-60.13,
-5.32,
-59.94,
-75.77,
-59.94,
-5.32,
-33.44,
-33.44,
-33.44,
-33.44,
-33.44,
-59.94,
-33.44,
-51.55,
-33.44,
-60.13,
-5.32,
-59.94,
-75.77,
-59.94,
-5.32,
-33.44,
-33.44,
-33.44,
-59.94,
-33.44,
-59.94,
-33.44,
-75.77,
-54.72,
-79.77,
-5.32,
-59.94,
-33.44,
-33.44,
-75.77,
-54.72,
-5.32,
-33.44,
-57.76,
-33.44,
-5.32,
-59.94,
-69.02,
-33.44,
-33.44,
-73.35,
-33.44,
-33.44,
-33.44,
-33.44,
-59.94,
-5.32,
-33.44,
-57.76,
-59.94,
-33.44,
-5.32,
-33.44,
-33.44,
-33.44,
-5.32,
-60.13,
-33.44,
-73.35,
-33.44,
-5.32,
-59.94,
-75.77,
-59.94,
-5.32,
-59.94,
-33.44,
-33.44,
-75.77,
-59.94,
-33.44,
-75.77,
-54.72,
-5.32,
-33.44,
-33.44,
-80.47,
-33.44,
-5.32,
-33.44,
-33.44,
-33.44,
-57.76,
-5.32,
-33.44,
-57.76,
-33.44,
-5.32,
-33.44,
-33.44,
-33.44,
-33.44,
-60.13,
-33.44,
-5.32,
-33.44,
-57.76,
-59.94,
-33.44,
-5.32,
-33.44,
-33.44,
-33.44,
-5.32,
-60.13,
-33.44,
-73.35,
-33.44,
-79.77,
-5.32,
-33.44,
-5.32,
-33.44,
-57.76,
-33.44,
-75.77,
-61.4,
-5.32,
-33.44,
-57.76,
-59.94,
-33.44,
-5.32,
-33.44,
-59.94,
-5.32,
-33.44,
-57.76,
-33.44,
-5.32,
-80.47,
-33.44,
-59.94,
-75.77,
-33.44,
-75.77,
-54.72,
-5.32,
-33.44,
-51.55,
-5.32,
-33.44,
-57.76,
-33.44,
-59.94,
-5.32,
-57.76,
-33.44,
-80.47,
-59.94,
-75.77,
-5.32,
-33.44,
-74.17,
-33.44,
-33.44,
-33.44,
-33.44,
-33.44,
-75.77,
-69.02,
-33.44,
-56.9
}};

/*
 * Function to go to predefined keyboard locations
 * to type out desired message on the screen.
 */
void runAutoState()
{
  for (int i = 0; i < 294; i++)
  {
    screenDotPos(0) = autoCoordinates[0][i];
    screenDotPos(2) = autoCoordinates[1][i];

    leftDotPos = gLS * screenDotPos;
    rightDotPos = gRS * screenDotPos;

    parallax();
    //Serial << screenDotPos;
    //Serial.println(" ");
    delay(1500);
  }

  state = MenuMode;
}
