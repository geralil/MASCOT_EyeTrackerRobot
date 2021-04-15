#include "Functions.h"

void setup() {
  // Set up Connection to Xbox controller
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXBOX USB Library Started"));

  // Initialize Limit Switch input pins
  pinMode(xEnd, INPUT); // x
  pinMode(yEnd, INPUT); //z
  pinMode(zEnd, INPUT); //y
  
  pinMode(41, OUTPUT); 
  digitalWrite(41, LOW); //Servo enable using relay - using that to prevent jitter when arduino starts
  XservoL.attach(xLPin);
  ZservoL.attach(zLPin);
  XservoR.attach(xRPin);
  ZservoR.attach(zRPin);
  
  // Set operating parameters for stepper motors
  stepperY.setMaxSpeed(10000); //SPEED = Steps / second
  stepperY.setAcceleration(1000); //ACCELERATION = Steps /(second)^2
  delay(500);
  stepperX.setMaxSpeed(10000); //SPEED = Steps / second
  stepperX.setAcceleration(1000); //ACCELERATION = Steps /(second)^2
  delay(500);
  stepperZ.setMaxSpeed(10000); //SPEED = Steps / second
  stepperZ.setAcceleration(1000); //ACCELERATION = Steps /(second)^2
  delay(500);
  
  InitialValues(); //averaging the values of the 3 analog pins (values from potmeters)
}

void loop() {
  Usb.Task();
  
  switch (state)
  {
    case (MenuMode):
      {
        runMenuModeState();
        break;
      }
    case (StepperHome):
      {
        runStepperHomeState();
        break;
      }
    case (StepperManual):
      {
        runStepperManualState();
        break;
      }
    case (ServoManual):
      {
        runServoManualState();
        break;
      }
    case (ServoCalibration):
      {
        runServoCalibrationState();
        break;
      }
    case (SetCoordinates):
      {
        //runSetCoordinatesState();
        break;
      }
    case (FindCoordinates):
      {
        //findCoordinatesState();
        break;
      }
    case (Auto):
      {
        //runAutoState();
        break;
      }
  }
  if (state != ServoManual)
  {
    delay(1);
  }
}
