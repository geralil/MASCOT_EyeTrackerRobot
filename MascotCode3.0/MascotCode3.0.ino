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

 

  XservoL.writeMicroseconds(centerLeftXMicroseconds);    // -ve left; +ve right
  ZservoL.writeMicroseconds(centerLeftZMicroseconds);    // -ve up; +ve down
  XservoR.writeMicroseconds(centerRightXMicroseconds);    // -ve left; +ve right
  ZservoR.writeMicroseconds(centerRightZMicroseconds);    // -ve down; +ve up
  
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
  ReadCalibrationVariablesFromProm();
}

 

int whichMotor = 0;

 

void loop() {
  Usb.Task();
//
//  if (Serial.available())
//  {
//    char c = Serial.read();
//    if( c=='a')
//      whichMotor++;
//    else if (c == 'z')
//      whichMotor--;
//    else if (c == 's')
//    {
//      if (whichMotor == 0)
//      {
//        centerLeftXMicroseconds += 5;
//      }
//      else if (whichMotor == 1)
//      {
//        centerLeftZMicroseconds += 5;
//      }
//      else if (whichMotor == 2)
//      {
//        centerRightXMicroseconds += 5;
//      }
//      else if (whichMotor == 3)
//      {
//        centerRightZMicroseconds += 5;
//      }
//    }
//    else if (c == 'x')
//    {
//      if (whichMotor == 0)
//      {
//        centerLeftXMicroseconds -= 5;
//      }
//      else if (whichMotor == 1)
//      {
//        centerLeftZMicroseconds -= 5;
//      }
//      else if (whichMotor == 2)
//      {
//        centerRightXMicroseconds -= 5;
//      }
//      else if (whichMotor == 3)
//      {
//        centerRightZMicroseconds -= 5;
//      }
//    }
//    else if (c == 'd')
//    {
//      if (whichMotor == 0)
//      {
//        centerLeftXMicroseconds += 1;
//      }
//      else if (whichMotor == 1)
//      {
//        centerLeftZMicroseconds += 1;
//      }
//      else if (whichMotor == 2)
//      {
//        centerRightXMicroseconds += 1;
//      }
//      else if (whichMotor == 3)
//      {
//        centerRightZMicroseconds += 1;
//      }
//    }
//    else if (c == 'c')
//    {
//      if (whichMotor == 0)
//      {
//        centerLeftXMicroseconds -= 1;
//      }
//      else if (whichMotor == 1)
//      {
//        centerLeftZMicroseconds -= 1;
//      }
//      else if (whichMotor == 2)
//      {
//        centerRightXMicroseconds -= 1;
//      }
//      else if (whichMotor == 3)
//      {
//        centerRightZMicroseconds -= 1;
//      }
//    }
//    else if( c == 'y')
//    {
//      calPointDotPos = !calPointDotPos;
//    }
//    else if( c == 'u')
//    {
//      screenDotPos = calPoint0;
//    }
//    else if( c == 'i')
//    {
//      screenDotPos = calPoint1;
//    }
//    else if( c == 'o')
//    {
//      screenDotPos = calPoint2;
//    }
//    else if( c == 'p')
//    {
//      screenDotPos = calPoint3;
//    }
//    else if( c == 'w')
//    {
//      WriteCalibrationVariablesToProm();
//    }
//    
//  }
  
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
        //runAutoState();
        break;
      }
  }
  if (state != ServoManual)
  {
    delay(1);
  }
}
