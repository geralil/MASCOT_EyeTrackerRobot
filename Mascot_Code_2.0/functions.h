#include <math.h>
#include <EEPROM.h>

/* 
 * Initialize all variables that are used throughout the code. 
 * Initializing here makes them "globally" known to all .h files
 * that are included below 
 * 
 */
float lookupRes=501;
float Asin_Table[501]={0};
float microsecondsPerDegree = 10.20408163;
 

// Initial values for screen dimensions
float screenWidth=260;
float screenHeight=175;
float eyeZlocation=95;

float z=0;
float x=0;

// Initial home angles for servos
float leftXhome=90;  //87.80
float leftZhome=90; //86
float rightXhome=90; //92.20
float rightZhome=90;  //86
float left90xhome = 0;
float left90zhome = 0;
float right90xhome = 0;
float right90zhome = 0;
float left90xmicro = 0;
float left90zmicro = 0;
float right90xmicro = 0;
float right90zmicro = 0;

float totalAngleXL = 0;
float totalAngleXR = 0;
float thetaXL = 0;
float thetaXR = 0;


//// Initial home angles for servos
//float leftXhome=89;
//float leftZhome=90;
//float rightXhome=91;
//float rightZhome=90;

float IPD=63; // This must change wth the Eye width
float deltaY=600;
float LxEye=0;
float LzEye=0;
float deltaX=0;
float deltaZ=0;
float leftLength=0;
float alphaLeft=0;
float betaLeft=0;
float RxEye=0;
float RzEye=0;
float rightLength=0;
float alphaRight=0;
float betaRight=0; 
float xStep=0;
float zStep=0;

// Initialize global screen coordinates. 
long bottomScreen=0;
long topScreen=0;
long leftScreen=0;
long rightScreen=0;
  
int i=0;
int xsteps=0; // Index of steps along the X Axis
int ysteps=1; // Index of steps along the Y Axis
int zsteps=2; // Index of steps along the Z Axis
int LSyAng=3; // Index of Left Yaw Angle 
int LSrAng=4; // Index of Left Roll Angle
int RSyAng=5; // Index of Right Yaw Angle
int RSrAng=6; // Index of Right Roll Angle
unsigned int Coordinates[7]={0,0,0,0,0,0,0}; // Initialization of Coodinate Array with all values set to 0
int xpos=0; // Index of dot position along the x axis
int zpos=1; // Index of dot position along the y axis
float DotPos[2]={0,0}; // Initialization of Dot Position Array with all values set to 0

/* These are the XY coordinates of the calibration points used by the Tobii Eye Tracker calibration
   routine. The calibrationCoords array will replace DotPos during the eye tracker calibration routine*/
float calibrationCoords[2][7]={{-2.15,-2.15,-95.75,104.98,100.55,-94.44,-3.21},{-30.49,-120.13,45.19,41.77,-130.50,-128.32,41.10}};

int threshold=3750; // Threshold for Xboc controller joystick inputs

//Pins
const byte Analog_X_pin = A0; //x-axis readings
const byte Analog_Y_pin = A1; //y-axis readings
const byte Analog_R_pin = A2; //r-axis readings
const byte LED_pin = 3; //PWM output for LED

//Variables
int Analog_X = 0; //x-axis value
int Analog_Y = 0; //y-axis value
int Analog_R = 0; //r-axis value
int Analog_Z=0;

int Analog_X_AVG = 0; //x-axis value average
int Analog_Y_AVG = 0; //y-axis value average
int Analog_R_AVG = 0; //r-axis value average

int Lim=75;

int del=0;
int joystick=0;
int joystick2=0;
float xStick=0;
float zStick=0;

//int steps=0;
float LxScreen=0;
float LzScreen=0;
float dir=0;
float dir2=0;
boolean limit=false;

int xEnd=23; // X axis limit switch pin
int zEnd=25; // Z axis limit switch pin
int yEnd=27; // Y axis limit switch pin

// Create State Machine
enum StateMachineState {
MenuMode   = 0,          // Press Start on X-BOX Controller
CAL    = 1,          // Press B on XBOX Controller
SERCAL = 2,          // Press A on XBOX Controller
AUTO   = 3,          // Press R3 on XBOX Controller
HOM    = 4,          // Press Y on XBOX Controller
STEPPERS = 5,        // Press RB on XBOX Controller
COORDINATES = 6,     // Press RT on XBOX Controller
SET_COORDINATES = 7, // Press LT on XBOX Controller 
MANUAL = 8,          // Press LB on XBOX Controller
};
StateMachineState state=MenuMode;


// Include the libraries for the XBOX Controller
#include <XBOXONE.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
// Initialize XBOX and USB Objects
USB Usb;
XBOXONE Xbox(&Usb);
int lim=7500;

// Include Accel Stepper Library and initialize stepper objects
#include <AccelStepper.h>
// Stepper Direction and Pulse Pins
int xdir=35;
int xpulse=33;
int ydir=37;//was 11
int ypulse=39;//was 8
int zdir=11;// was 2
int zpulse=8;//was 3

/* Conversion factor for converting stepper motor steps to milimeters. This value was experimentally determined
 *  for each axis by making the axis move by a set amount of steps and then measuring the actual distance traveled.
 *  Taking the ratio between steps traveled and mm traveled leaves us with StepsPerMM.
*/
float StepsPerMM=80.22; //was 80.22

/* Initialize AccelStepper objects for each stepper motor.
 * AccelStepper is an amazing library that offers a ton of great features for stepper control.
 * 
 * Extra documentation and functions can be found here. https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
 */
AccelStepper stepperX(AccelStepper::DRIVER, xpulse, xdir); // X Axis Stepper
AccelStepper stepperY(AccelStepper::DRIVER, ypulse, ydir); // Y Axis Stepper
AccelStepper stepperZ(AccelStepper::DRIVER, zpulse, zdir); // Z Axis Stepper
 
// Create Servo Objects
#include <Servo.h>
Servo XservoL; // Left X Servo (Looks left and right)
Servo ZservoL; // Left Z Servo (Looks up and down)
Servo XservoR; // Right X Servo (Looks left and right)
Servo ZservoR; // Right Z Servo (Looks up and down)


// Inlcude all functions files
#include "distancePerStep.h" // Funtions used to find the distance per step
#include "auto_functions.h"  // Nothing added yet. Was originally meant to contain all auto routines
#include "manual_functions.h" // Not currently in use. Contains the getMove() function which has been replaced by GetPos().  
#include "homing_functions.h" // Contains zeroStepper() which is used to home the stepper axes.
#include "Steppers.h" // Contains all functions needed to manually control the stepepr axes.
#include "Servos.h" // Contains all functions needed to manually control the servos.
#include "calibration.h" // Contains all functions needed to calibrate the steppers and the servos to the rig.
//#include "James_functions.h" // Not currently in use. The idea was to use these files as "storage" for conceptual functions
//#include "Leif_functions.h"  // Not currently in use. The idea was to use these files as "storage" for conceptual functions


// The initial state after system startup
void runMenuModeState()
{
  //Conditional statements to change the state if desired.
  if(Xbox.getButtonClick(Y))
  {
    state=HOM; 
  }
  if(Xbox.getButtonClick(A))
  {
    state=SERCAL;
  }
  if(Xbox.getButtonClick(R3))
  {
    state=AUTO;
  }
  if(Xbox.getButtonClick(B))
  {
    state=CAL;
  }
    if(Xbox.getButtonClick(R2))
  {
    state=COORDINATES;
  }
  if(Xbox.getButtonClick(L2))
  {
    state=SET_COORDINATES;
  }
  if(Xbox.getButtonClick(R1))
  {
    state=STEPPERS;
  }
  if(Xbox.getButtonClick(L1))
  {
    state=MANUAL;
  }
}

void LoadCalibrationVariablesToProm()
{
  int eeAddress = 0;

  EEPROM.put(eeAddress, left90xmicro);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, left90zmicro);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, right90xmicro);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, right90zmicro);
  eeAddress += sizeof(float);

  EEPROM.put(eeAddress, leftScreen);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, rightScreen);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, bottomScreen);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, topScreen);
  delay(100);

  Serial.println("Variables loaded!");
}

void ReadCalibrationVariablesFromProm()
{
  int eeAddress = 0;
  
  EEPROM.get(eeAddress, left90xmicro);
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, left90zmicro);
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, right90xmicro);
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, right90zmicro);
  eeAddress += sizeof(float);
  
  EEPROM.get(eeAddress, leftScreen);
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, rightScreen);
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, bottomScreen);
  eeAddress += sizeof(float);
  EEPROM.get(eeAddress, topScreen);

  Serial.println("Variables read!");
}


/*  This function will run through the entire calibration routine for the rig. 
 *   
 *  Make sure to mount both eye lasers, and the chin laser on the rig. 
 *   
 *  First the stepper motors are calibrated via the stepperCalibration() function. 
 *  This is done by homing each axis towards its respective limit switch. Then manual 
 *  control is given to the user so he/she can pilot the chin laser to the bottom left 
 *  and top right corners of the screen. Once this is complete, the rig moves the chin 
 *  laser to the center of the screen and returns from stepperCalibration(). 
 *  
 *  servoCalibration() then gives manual control to first the left and then the
 *  right eye so they can be steered to converege with the chin laser. Once this is complete, 
 *  then the rig is fully calibrated and can be calibrated to the eye tracker. 
*/
void runCalState()
{
stepperCalibration();

state=MenuMode;
}

void runSerCalState()
{
  servoCalibration();
  LoadCalibrationVariablesToProm();

  state = MenuMode;
}

// This function will allow the eyes to be controlled manually
void runManualState()
{
  GetPos(); // Moves a virtual dot around the screen via the left joystick.
  parallax(); // Contains the relationshps to make the eyes converge on a single point.

  //Conditional statements to change the state if desired.
   if(Xbox.getButtonClick(Y))
  {
    state=HOM;
  }
  if(Xbox.getButtonClick(B))
  {
    state=CAL;
  }
  if(Xbox.getButtonClick(R3))
  {
    state=AUTO;
  }
  if(Xbox.getButtonClick(START))
  {
    state=MenuMode;
  }
  if(Xbox.getButtonClick(R1))
  {
    state=STEPPERS;
  }
    if(Xbox.getButtonClick(R2))
  {
    state=COORDINATES;
  }
  if(Xbox.getButtonClick(L2))
  {
    state=SET_COORDINATES;
  }  
  if(Xbox.getButtonClick(A))
  {
    state = SERCAL;
  }
}

// No current use. May be used in the future to run more complex automated routines
void runAutoState()
{
  //Conditional statements to change the state if desired.
   if(Xbox.getButtonClick(Y))
  {
    state=HOM;
  }
  if(Xbox.getButtonClick(B))
  {
    state=CAL;
  }
  if(Xbox.getButtonClick(A))
  {
    state=SERCAL;
  }
  if(Xbox.getButtonClick(START))
  {
    state=MenuMode;
  }
    if(Xbox.getButtonClick(R2))
  {
    state=COORDINATES;
  }
  if(Xbox.getButtonClick(L2))
  {
    state=SET_COORDINATES;
  }
  if(Xbox.getButtonClick(L1))
  {
    state = MANUAL;
  }
}

/* This function will home the steppers and reset the home location.
 *  
 *  NOTE: DO NOT change the order of X and Z homing routines. If they are swapped, 
 *  there is a chance that the rig may crash.
*/
void runHomeState()
{
  zeroStepper(xdir,xpulse,xEnd); // DO NOT switch with Z
  stepperX.setCurrentPosition(-500);
  zeroStepper(ydir,ypulse,yEnd);
  stepperY.setCurrentPosition(500);
  zeroStepper(zdir,zpulse,zEnd); // DO NOT switch with X
  stepperZ.setCurrentPosition(500);
  state=MenuMode;
}

// Allows for manual control of the stepper axes
void runSteppersState()
{
  i=0;

  while(i==0) // Keep the system in the stepper state
{
  
  Usb.Task(); // Request data from Xbox controller
  ReadAnalog(); // Take Xbox controller data and convert to desired stepper axis movement
  stepperX.run(); 
  stepperY.run();
  stepperZ.run();
  
  //Conditional statements to change the state if desired.
  if(Xbox.getButtonClick(Y))
  {
    state=HOM;
    i=1;
  }
  if(Xbox.getButtonClick(B))
  {
    state=CAL;
    i=1;
  }
  if(Xbox.getButtonClick(A))
  {
    state=SERCAL;
    i=1;
  }
  if(Xbox.getButtonClick(START))
  {
    state=MenuMode;
    i=1;
  } 
    if(Xbox.getButtonClick(R2))
  {
    state=COORDINATES;
  }
  if(Xbox.getButtonClick(L2))
  {
    state=SET_COORDINATES;
  }
  if(Xbox.getButtonClick(L1))
  {
    state = MANUAL;
    i=1;
  }
}  
}

void LookupTables()
{
  // Arcsin
  for(i=0;i<lookupRes;i++)
  {
    Asin_Table[i]=asin((i*(2/(lookupRes-1))-.5));
  }

}
