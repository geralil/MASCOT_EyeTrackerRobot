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


  gBC = xform(-stepperX.currentPosition() / StepsPerMM,
              -stepperY.currentPosition() / StepsPerMM,
              stepperZ.currentPosition() / StepsPerMM);

  gLS = gLD.Inverse() * gCD.Inverse() * gBC.Inverse() * gBS;

  gRS = gRD.Inverse() * gCD.Inverse() * gBC.Inverse() * gBS;

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

//  Serial << leftDotPos;
//  Serial.println();
//  Serial << rightDotPos;
//  Serial.println();

  // calculating angles of rotation for the right eye
  alphaRight  = atan2(rightDotPos(1), rightDotPos(0)) * (180 / 3.14159);
  betaRight = atan2(rightDotPos(2), sqrt((rightDotPos(0) * rightDotPos(0)) + (rightDotPos(1) * rightDotPos(1)))) * (180 / 3.14159);

  Serial.print("alphaLeft");
  Serial.println(alphaLeft);
  Serial.print("betaLeft");
  Serial.println(betaLeft);
  Serial.print("alphaRight");
  Serial.println(alphaRight);
  Serial.print("betaRight");
  Serial.println(betaRight);

  // writing the angle values to the X and Z servos.
  XservoL.writeMicroseconds(1500 + (90 - alphaLeft) * microsecondsPerDegree);
  ZservoL.writeMicroseconds(1500 + (betaLeft) * microsecondsPerDegree);
  XservoR.writeMicroseconds(1500 + (90 - alphaRight) * microsecondsPerDegree);
  ZservoR.writeMicroseconds(1500 + (betaRight) * microsecondsPerDegree);
}
