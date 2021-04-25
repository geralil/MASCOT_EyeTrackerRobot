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

// Include this library for matrix manipulation
#include <BasicLinearAlgebra.h>
// Library for writing and retreiving information using EEPROM
#include <EEPROM.h>

// Library for Nextion screen and GUI implementation
#include <SoftwareSerial.h>
#include <Nextion.h>

// Create State Machine
enum StateMachineState {
  MenuMode = 0,         // Press Start on X-BOX Controller
  ServoCalibration = 1, // Press A on XBOX Controller
  Auto = 2,             // Press R3 on XBOX Controller
  StepperHome = 3,      // Press Y on XBOX Controller
  StepperManual = 4,    // Press RB on XBOX Controller
  FindCoordinates = 5,  // Press RT on XBOX Controller
  SetCoordinates = 6,   // Press LT on XBOX Controller
  ServoManual = 7,      // Press LB on XBOX Controller
};
StateMachineState state = MenuMode;

//Setup Serial port for screen (TX,RX) 
SoftwareSerial mySerial(36, 37);

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

/*
    Now initializing limit switch pins, stepper direction and pulse pins, servo pins
    Stepper objects and servo objects will also be defined
*/

// Declare Servo Pins - check wiring diagram if you have any questions
int xLPin = 4;
int zLPin = 5;
int xRPin = 6;
int zRPin = 7;

int xEnd = 23; // X axis limit switch pin
int zEnd = 25; // Z axis limit switch pin
int yEnd = 27; // Y axis limit switch pin

boolean limit = false;  // default value of limit switch reading

// Stepper Direction and Pulse Pins
int xdir = 35;
int xpulse = 33;
int ydir = 37;
int ypulse = 39;
int zdir = 11;
int zpulse = 8;

/* Initialize AccelStepper objects for each stepper motor.
   AccelStepper is an amazing library that offers a ton of great features for stepper control.

   Extra documentation and functions can be found here. https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
*/
AccelStepper stepperX(AccelStepper::DRIVER, xpulse, xdir); // X Axis Stepper
AccelStepper stepperY(AccelStepper::DRIVER, ypulse, ydir); // Y Axis Stepper
AccelStepper stepperZ(AccelStepper::DRIVER, zpulse, zdir); // Z Axis Stepper

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
float leftMicrosecondsPerDegree = 10.20408163*1.06;
float rightMicrosecondsPerDegree = 10.20408163;

float leftXMicrosceondsPerDegree = 10.20408163*1.01;

long int centerLeftXMicroseconds = 1500;
long int centerLeftZMicroseconds = 1500;
long int centerRightXMicroseconds = 1500;
long int centerRightZMicroseconds = 1500;

int calMotor = 0;

float xStick = 0;
float zStick = 0;

float LrotationY = 0; //-M_PI/200
float RrotationY = 0; //1*(M_PI/200)

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
   Function to create a translation only transformation matrix using the X,Y,Z
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

/*
  This is the distance between our origin coordinate system B
  (when steppers hit the limit switches
  + offsets to the center of the stage) to the origin of the screen S.
*/
BLA::Matrix<4, 4> gBS = xform(0,0,0,(141.23), 562, (231.23)); //140.15, 220.46
//BLA::Matrix<4,4> gBS = {1,0,0,141.23,
//                        0,0.9659,0.2588,562,
//                        0,-0.2588,0.9659,231.23,
//                        0,0,0,1};
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
BLA::Matrix<4, 4> gCD = xform(0,0,0,0, 89.41, 116.72);
//BLA::Matrix<4,4> gCD =  {1,0,0,0,
//                         0,1,0,92.18,
//                         0,0,1,96.07,
//                         0,0,0,1};

/*
   This is the distance between our D coordinate system to the center of the left eye L.
*/
BLA::Matrix<4, 4> gLD = xform(0,LrotationY,0,(-34.75), 0, 0);
//BLA::Matrix<4,4> gLE = {1,0,0,-33.47,
//                        0,1,0,0,
//                        0,0,1,0,
//                        0,0,0,1};
/*
   This is the distance between our D coordinate system to the center of the right eye R.
*/
BLA::Matrix<4, 4> gRD = xform(0,RrotationY,0,(34.75), 0, 0);
//BLA::Matrix<4,4> gRE = {1,0,0,33.47,
//                        0,1,0,0,
//                        0,0,1,0,
//                        0,0,0,1};

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
#include "GUI.h"

/*
 * Function to update the transformation matrices
 */
void UpdateTransformation()
{
  // Recalculating gBC and gLS, gRS
  gBC = xform(0,0,0,-stepperX.currentPosition() / StepsPerMM,
                              -stepperY.currentPosition() / StepsPerMM,
                              stepperZ.currentPosition() / StepsPerMM);
  gLS = gLD.Inverse() * gCD.Inverse() * gBC.Inverse() * gBS;

  gRS = gRD.Inverse() * gCD.Inverse() * gBC.Inverse() * gBS;
  
  Serial.println("Transformation Updated");
}

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
  
  Serial.println("Variables read!");
  UpdateTransformation();
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
  servoCalibration();
  parallax();
  
  if (IfButtonPressed() == true)
  {
    WriteCalibrationVariablesToProm();
  }
}

/*
   Servo manual state will be used to control the eye servos manually using the XBOX controller.
*/
void runServoManualState()
{
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
  zeroStepper(xdir, xpulse, xEnd); // DO NOT switch with Z
  stepperX.setCurrentPosition(-500);
  zeroStepper(ydir, ypulse, yEnd);
  stepperY.setCurrentPosition(500);
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

void InitialValues()
{
  //Set the values to zero before averaging
  float tempX = 0;
  float tempY = 0;
  float tempR = 0;
  //----------------------------------------------------------------------------
  //read the analog 50x, then calculate an average.
  //they will be the reference values
  for (int i = 0; i < 50; i++)
  {
    Usb.Task();
    //     tempX += analogRead(Analog_X_pin);
    tempX += map(Xbox.getAnalogHat(LeftHatX), -32767, 32767, 0, 1023);

    delay(10); //allowing a little time between two readings
    //     tempY += analogRead(Analog_Y_pin);
    tempY += map(Xbox.getAnalogHat(LeftHatY), -32767, 32767, 0, 1023);
    delay(10);
    tempR += map(Xbox.getAnalogHat(RightHatX), -32767, 32767, 0, 1023);
    delay(10);
  }
  //----------------------------------------------------------------------------
  Analog_X_AVG = tempX / 50;
  Analog_Y_AVG = tempY / 50;
  Analog_R_AVG = tempR / 50;
  //----------------------------------------------------------------------------
  Serial.print("AVG_X: ");
  Serial.println(Analog_X_AVG);
  Serial.print("AVG_Y: ");
  Serial.println(Analog_Y_AVG);
  Serial.print("AVG_R: ");
  Serial.println(Analog_R_AVG);
  Serial.println("Calibration finished");
}
