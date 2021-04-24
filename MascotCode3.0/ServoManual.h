/*
   Function to use input from Xbox controller and convert to coordinates
   that control the gaze of the eyes on the screen. Updates screenDotPos matrix
   with the new coordinates. Must be called before parallax.
*/
void GetScreenDotPosition()
{
  int lim = 7500;
  if (Xbox.XboxOneConnected) {
    if (Xbox.getAnalogHat(LeftHatX) > lim || Xbox.getAnalogHat(LeftHatX) < -lim || Xbox.getAnalogHat(LeftHatY) > lim || Xbox.getAnalogHat(LeftHatY) < -lim || Xbox.getAnalogHat(RightHatX) > lim || Xbox.getAnalogHat(RightHatX) < -lim || Xbox.getAnalogHat(RightHatY) > lim || Xbox.getAnalogHat(RightHatY) < -lim) {

      xStick = Xbox.getAnalogHat(LeftHatX);
      zStick = Xbox.getAnalogHat(LeftHatY);

      if (calPointDotPos == 0)
      {
        if (Xbox.getAnalogHat(LeftHatX) > lim || Xbox.getAnalogHat(LeftHatX) < -lim) {

          screenDotPos(0) += (xStick / (32767));
        }
        if (Xbox.getAnalogHat(LeftHatY) > lim || Xbox.getAnalogHat(LeftHatY) < -lim) {

          screenDotPos(2) += (zStick / (32767));
        }
        if (Xbox.getAnalogHat(RightHatX) > lim || Xbox.getAnalogHat(RightHatX) < -lim) {

        }
        if (Xbox.getAnalogHat(RightHatY) > lim || Xbox.getAnalogHat(RightHatY) < -lim) {

        }
      }
    }
  }

  leftDotPos = gLS * screenDotPos;
  rightDotPos = gRS * screenDotPos;
}

/*
   Parallax function that will calculate the angles of rotation for the servos.
   The function writes to the servos as well, in order to make them converge on a single
   point. The angles are calculated in terms of the azimuth and the elevation of the dot position
   from the 90-degree position of the eyes. Azimuth and elevation can be given as:
   azimuth = atan2(y,x)
   elevation = atan2(z,sqrt(x.^2 + y.^2))
*/
void parallax() {

  // calculating angles of rotation for the left eye
  alphaLeft = atan2(leftDotPos(1), leftDotPos(0)) * (180 / 3.14159);
  betaLeft = -atan2(leftDotPos(2), sqrt((leftDotPos(0) * leftDotPos(0)) + (leftDotPos(1) * leftDotPos(1)))) * (180 / 3.14159);

  // calculating angles of rotation for the right eye
  alphaRight  = atan2(rightDotPos(1), rightDotPos(0)) * (180 / 3.14159);
  betaRight = atan2(rightDotPos(2), sqrt((rightDotPos(0) * rightDotPos(0)) + (rightDotPos(1) * rightDotPos(1)))) * (180 / 3.14159);
  
  if ((alphaLeft != alphaLeftPrev) || (betaLeft != betaLeftPrev) || (state == ServoCalibration))
  {
    // writing the angle values to the X and Z servos.
    XservoL.writeMicroseconds(centerLeftXMicroseconds + (90 - alphaLeft) * leftXMicrosceondsPerDegree);
    //delay(50);
    ZservoL.writeMicroseconds(centerLeftZMicroseconds + (betaLeft) * leftMicrosecondsPerDegree);
    //delay(50);
    XservoR.writeMicroseconds(centerRightXMicroseconds + (90 - alphaRight) * microsecondsPerDegree);
    //delay(50);
    ZservoR.writeMicroseconds(centerRightZMicroseconds + (betaRight) * microsecondsPerDegree);
    //delay(50);
  
    alphaLeftPrev = alphaLeft;
    betaLeftPrev = betaLeft;
  }
}


void servoCalibration()
{
  screenDotPos(0) = 0;
  screenDotPos(2) = 0;

  leftDotPos = gLS * screenDotPos;
  rightDotPos = gRS * screenDotPos;

  if (Xbox.getButtonClick(RIGHT))
  {
    calMotor = 4;
  }
  if (Xbox.getButtonClick(LEFT))
  {
    calMotor = 3;
  }
  if (Xbox.getButtonClick(UP))
  {
    calMotor = 2;
  }
  if (Xbox.getButtonClick(DOWN))
  {
    calMotor = 1;
  }

  int lim = 7500;
 
  if (Xbox.XboxOneConnected) {
    if (Xbox.getAnalogHat(LeftHatX) > lim || Xbox.getAnalogHat(LeftHatX) < -lim || Xbox.getAnalogHat(LeftHatY) > lim || Xbox.getAnalogHat(LeftHatY) < -lim || Xbox.getAnalogHat(RightHatX) > lim || Xbox.getAnalogHat(RightHatX) < -lim || Xbox.getAnalogHat(RightHatY) > lim || Xbox.getAnalogHat(RightHatY) < -lim) {

      xStick = Xbox.getAnalogHat(LeftHatX);
      zStick = Xbox.getAnalogHat(LeftHatY);

      if (Xbox.getAnalogHat(LeftHatX) > lim || Xbox.getAnalogHat(LeftHatX) < -lim) {
        switch (calMotor)
        {
          case (4) :{ centerRightXMicroseconds += ((xStick) / 32767);
            break;
          }
          case (3) :{ centerLeftXMicroseconds += ((xStick) / 32767);
            break;
          }
          default : {
            break;
          }
        }

      }
      if (Xbox.getAnalogHat(LeftHatY) > lim || Xbox.getAnalogHat(LeftHatY) < -lim) {
        switch (calMotor)
        {
          case (1) :{ centerRightZMicroseconds += ((zStick) / 32767);
            break;
          }
          case (2) :{ centerLeftZMicroseconds += ((zStick) / 32767);
            break;
          }
          default : {
            break;
          }
        }
      }      
    }
  }
}
