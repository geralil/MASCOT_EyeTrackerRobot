#include "Functions.h"

 

void setup() {
  // Set up Connection to Xbox controller and BNO IMU
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  Serial.println("Serial On");
  if(!bno.begin())
  {
   Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  
  bno.setExtCrystalUse(true);   // prepping the IMU
  
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
  
  pinMode(8, OUTPUT); 
  digitalWrite(8, LOW); //Servo enable using relay - using that to prevent jitter when arduino starts
  
  // attaching servos to their respective pins.//
  XservoL.attach(xLPin);
  ZservoL.attach(zLPin);
  XservoR.attach(xRPin);
  ZservoR.attach(zRPin);
  NeckServo.attach(NSPin);

  // Loading calibration variables for neck stepper from EEprom
  //ReadCalibrationStepperPosFromProm();
  //ReadLastStepperPosFromProm();

  /* 
   * Set operating parameters for neck stepper motors 
   * stepper one is front
   * stepper two is back right
   * stepper three is back left
   */
  stepperOne.setCurrentPosition(laststepperposOne); //laststepperposOne); //knownstepperposOne);
  stepperOne.setMaxSpeed(3000); //SPEED = Steps / second 
  stepperOne.setAcceleration(3000); //ACCELERATION = Steps /(second)^2    
  delay(500);
  stepperTwo.setCurrentPosition(laststepperposTwo); //laststepperposTwo); //knownstepperposTwo);
  stepperTwo.setMaxSpeed(3000); //SPEED = Steps / second 
  stepperTwo.setAcceleration(3000); //ACCELERATION = Steps /(second)^2
  delay(500); 
  stepperThree.setCurrentPosition(laststepperposThree); //laststepperposThree); //knownstepperposThree); 
  stepperThree.setMaxSpeed(3000); //SPEED = Steps / second 
  stepperThree.setAcceleration(3000); //ACCELERATION = Steps /(second)^2      
  delay(500);

  // Loading calibration variables for eye servos and shoulder steppers from Prom
  ReadCalibrationVariablesFromProm();

  XservoL.writeMicroseconds(centerLeftXMicroseconds);    // -ve left; +ve right
  ZservoL.writeMicroseconds(centerLeftZMicroseconds);    // -ve up; +ve down
  XservoR.writeMicroseconds(centerRightXMicroseconds);    // -ve left; +ve right
  ZservoR.writeMicroseconds(centerRightZMicroseconds);    // -ve down; +ve up
  NeckServo.write(83);
  
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

 

int whichMotor = 0;

 

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
        runSetCoordinatesState();
        break;
      }
    case (FindCoordinates):
      {
        runFindCoordinatesState();
        break;
      }
    case (Auto):
      {
        runAutoState();
        break;
      }
    case (NeckCalibration):
      {
        runNeckCalibrationState();
        break;
      }
    case (MoveToCalibrationState):
      {
        runMoveToCalibrationState();
        break;
      }
    case (Neck):
      {
        runNeckState();
        break;
      }
  }
  if (state != ServoManual || state != NeckCalibration)
  {
    delay(1);
  }
}
