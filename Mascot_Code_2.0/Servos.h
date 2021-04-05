/*
Float map not used anymore.
Maps the signal coming from the XBOX to a range acceptable to Servos
*/
float floatMap(float x, float in_min, float in_max, float out_min, float out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* 
GetPos() returns the updated DotPos array with coordinates for
virtual dot on the screen. 
Used by parallax, lefteye, righteye functions
*/
int GetPos()
{
  float positiveXLim = 130;
  float negativeXLim = -130;
  float positiveZLim = 170;
  float negativeZLim = -170;
  float rollAngle = 36;

//  Serial.print("StepperX: ");
//  Serial.println(stepperX.currentPosition());
//  Serial.print("StepperZ: ");
//  Serial.println(stepperZ.currentPosition());
  

  if (stepperX.currentPosition() != 0)
  {
    positiveXLim += stepperX.currentPosition()/StepsPerMM;
    negativeXLim += stepperX.currentPosition()/StepsPerMM;
  }
  
  if (stepperZ.currentPosition() != 0)
  {
    positiveZLim += stepperZ.currentPosition()/StepsPerMM;
    negativeZLim += stepperZ.currentPosition()/StepsPerMM;
  }

  if (rollAngle != 0)
  {
    positiveXLim = positiveXLim*cos(rollAngle) - positiveZLim*sin(rollAngle);
    positiveZLim = positiveXLim*sin(rollAngle) - positiveZLim*cos(rollAngle);

    negativeXLim = negativeXLim*cos(rollAngle) - negativeZLim*sin(rollAngle);
    negativeZLim = negativeXLim*sin(rollAngle) - negativeZLim*cos(rollAngle);
//
//    DotPos[xpos] = DotPos[xpos]*cos(rollAngle) - DotPos[zpos]*sin(rollAngle);
//    DotPos[zpos] = DotPos[xpos]*sin(rollAngle) - DotPos[zpos]*cos(rollAngle);
  }

//  Serial.print("positiveXlim: ");
//  Serial.println(positiveXLim);
//  Serial.print("positiveZlim: ");
//  Serial.println(positiveZLim);
  
  
    if (Xbox.XboxOneConnected) {
    if (Xbox.getAnalogHat(LeftHatX) > lim || Xbox.getAnalogHat(LeftHatX) < -lim || Xbox.getAnalogHat(LeftHatY) > lim || Xbox.getAnalogHat(LeftHatY) < -lim || Xbox.getAnalogHat(RightHatX) > lim || Xbox.getAnalogHat(RightHatX) < -lim || Xbox.getAnalogHat(RightHatY) > lim || Xbox.getAnalogHat(RightHatY) < -lim) {

       xStick=Xbox.getAnalogHat(LeftHatX);
       zStick=Xbox.getAnalogHat(LeftHatY);
      
      if (Xbox.getAnalogHat(LeftHatX) > lim || Xbox.getAnalogHat(LeftHatX) < -lim) {
    
        DotPos[xpos]+=(xStick/(65534));

        if (DotPos[xpos] > positiveXLim)
        {
          DotPos[xpos] = positiveXLim;
        }
        if (DotPos[xpos] < negativeXLim)
        {
          DotPos[xpos] = negativeXLim; 
        }
      }
      if (Xbox.getAnalogHat(LeftHatY) > lim || Xbox.getAnalogHat(LeftHatY) < -lim) {
      
        DotPos[zpos]+=(zStick/(65534));
        
        if (DotPos[zpos] > positiveZLim)
        {
          DotPos[zpos] = positiveZLim;
        }
        if (DotPos[zpos] < negativeZLim)
        {
          DotPos[zpos] = negativeZLim; 
        }        
      }
      if (Xbox.getAnalogHat(RightHatX) > lim || Xbox.getAnalogHat(RightHatX) < -lim) {
       
      }
      if (Xbox.getAnalogHat(RightHatY) > lim || Xbox.getAnalogHat(RightHatY) < -lim) {      
     
      }
    }
  }
}

/*
MoveDot not used anymore. Performs same functionality as GetPos()
*/ 
int MoveDot()
{
  // Moves the dot around the 2D screen. Larger stick inputs will result in a faster position change ie. distance increment per loop. 

  if(abs(xStick)>threshold) //Moves left or right
  {
    DotPos[xpos]+=map(xStick,-32767,32767,-1,1);
    if(DotPos[xpos]>292.1)
    DotPos[xpos]=592.1;
    if(DotPos[xpos]<0)
    DotPos[xpos]=-292;
  }

  if(abs(zStick)>threshold) //Moves up or down
  {
    DotPos[zpos]+=map(zStick,-32767,32767,-1,1);
    if(DotPos[zpos]>200.25)
    DotPos[zpos]=200.25;
    if(DotPos[zpos]<0)
    DotPos[zpos]=0;
  }
}

/*
Main function that gets called right after GetPos()
Purpose is to move the servos to the positions of the dot
informed by DOTPOS array.
*/
void parallax(){
  
//  xStep=(abs(stepperX.currentPosition())-leftScreen)/StepsPerMM;
//  zStep=(abs(stepperZ.currentPosition())-bottomScreen)/StepsPerMM;

//  xStep=(abs(stepperX.currentPosition()))/StepsPerMM;
//  zStep=(abs(stepperZ.currentPosition()))/StepsPerMM;

  xStep = 0;
  zStep = 0;
//DotPos[0] = 0;
//DotPos[1] = 0;

//  Serial.print("DotPos[0] :");
//  Serial.println(DotPos[0]);
//  Serial.print("DotPos[1] :");
//  Serial.println(DotPos[1]);
  
  //Left Eye position on screen
  LxEye = xStep-(0.5*IPD);  
  LzEye = zStep + eyeZlocation;

  //distance from the eyes to the virtual dot
  deltaX = DotPos[0]-LxEye;
  deltaZ = DotPos[1]-LzEye;

  //Euclidean distance to the dot position from the left eye
  leftLength = sqrt(deltaX*deltaX+deltaZ*deltaZ+deltaY*deltaY);

  //totalAngleXL = atan(deltaX/deltaY) * (180/3.14159);
  //angle to the dot from home position
  alphaLeft = atan(deltaX/deltaY)*(180/3.14159);
  betaLeft = asin(deltaZ/leftLength)*(180/3.14159);

  // thetaXL = totalAngleXL - (leftXhome-left90xhome);
   //Serial.println(thetaXL);
  
  //Right Eye
  RxEye = xStep+(0.5*IPD);  
  RzEye = zStep + eyeZlocation;

  deltaX = DotPos[0]-RxEye;
  deltaZ = DotPos[1] -RzEye;

  rightLength = sqrt(deltaX*deltaX+deltaZ*deltaZ+deltaY*deltaY);

  //totalAngleXR = atan(deltaX/deltaY) * (180/3.14159);
  //betaRight = -betaLeft - 16.5; //this is right
  betaRight = -asin(deltaZ/rightLength) * (180/3.14159) - 16.13;
  alphaRight = atan(deltaX/deltaY)*(180/3.14159);

    //thetaXR = totalAngleXR + (rightXhome-right90xhome);
    
//  Serial.println("betaleft ");
//  Serial.println(betaLeft);
//  Serial.println("betaright ");
//  Serial.println(betaRight);
//    Serial.println("alphaleft ");
//  Serial.println(alphaLeft);
//  Serial.println("alpharight ");
//  Serial.println(alphaRight);
//  
//  // Write New Positions to Servos
//  XservoL.write(leftXhome+thetaXL);
//  ZservoL.write(leftZhome+betaLeft);
//  
//  XservoR.write(rightXhome+thetaXR);
//  ZservoR.write(rightZhome+betaRight);
//
//  delay(0.5);
//  i=0;

//  float leftXhomeMicro = left90xmicro+ (leftXhome - left90xhome) *microsecondsPerDegree;
//  float leftZhomeMicro = left90zmicro+ (leftZhome - left90zhome) *microsecondsPerDegree;;
//  float rightXhomeMicro = right90xmicro+ (rightXhome - right90xhome) *microsecondsPerDegree;;
//  float rightZhomeMicro = right90zmicro+ (rightZhome - right90zhome) *microsecondsPerDegree;;

  float leftXhomeMicro = left90xmicro;
  float leftZhomeMicro = left90zmicro;
  float rightXhomeMicro = right90xmicro;
  float rightZhomeMicro = right90zmicro;

//  XservoL.writeMicroseconds(leftXhomeMicro + thetaXL * microsecondsPerDegree);
//  ZservoL.writeMicroseconds(leftZhomeMicro + betaLeft * microsecondsPerDegree);
//  XservoR.writeMicroseconds(rightXhomeMicro + thetaXR * microsecondsPerDegree);
//  ZservoR.writeMicroseconds(rightZhomeMicro + betaRight * microsecondsPerDegree);

  XservoL.writeMicroseconds(leftXhomeMicro + alphaLeft * microsecondsPerDegree);
  ZservoL.writeMicroseconds(leftZhomeMicro + betaLeft * microsecondsPerDegree);
  XservoR.writeMicroseconds(rightXhomeMicro + alphaRight * microsecondsPerDegree);
  ZservoR.writeMicroseconds(rightZhomeMicro + betaRight * microsecondsPerDegree);

//  XservoL.writeMicroseconds(1500 + alphaLeft * microsecondsPerDegree);
//  ZservoL.writeMicroseconds(1500 + betaLeft * microsecondsPerDegree);
//  XservoR.writeMicroseconds(1500 + alphaRight * microsecondsPerDegree);
//  ZservoR.writeMicroseconds(1500 + betaRight * microsecondsPerDegree);

  delay(1);

//    Serial.println("leftXhomeMicros ");
//  Serial.println(XservoL.readMicroseconds());
//    Serial.println("LeftZhomeMicros ");
//  Serial.println(ZservoL.readMicroseconds());
//      Serial.println("rightXomeMicros ");
//  Serial.println(XservoR.readMicroseconds());
//    Serial.println("rightZhomeMicros ");
//  Serial.println(ZservoR.readMicroseconds());
//  for (float i = -10; i< 10; i += 0.25)
//  {
//    XservoL.writeMicroseconds(i*microsecondsPerDegree +1500);
//    delay(1000);
//  }
}

void leftEye()
{  
  //Left Eye
  LxEye = xStep-(0.5*IPD);  
  LzEye = zStep+eyeZlocation;

  deltaX = DotPos[0]-LxEye;
  deltaZ = DotPos[1]-LzEye;

  leftLength = sqrt(deltaX*deltaX+deltaZ*deltaZ+deltaY*deltaY);

  alphaLeft = asin(deltaX/leftLength)*(180/3.14159);
  betaLeft = asin(deltaZ/leftLength)*(180/3.14159);

  

  XservoL.writeMicroseconds(alphaLeft*microsecondsPerDegree + 1500);
  ZservoL.writeMicroseconds(betaLeft*microsecondsPerDegree + 1500);

  
  delay(2);
  
//  Serial.println("XSERVOL");
//  Serial.println(XservoL.read());
//  Serial.println("ZSERVOL");
//  Serial.println(ZservoL.read());
}

void rightEye()
{
  RxEye = xStep+(0.5*IPD);  
  RzEye = zStep+eyeZlocation;

  deltaX = DotPos[0]-RxEye;
  deltaZ = DotPos[1]-RzEye;

  rightLength = sqrt(deltaX*deltaX+deltaZ*deltaZ+deltaY*deltaY);

  alphaRight = asin(deltaX/rightLength)*(180/3.14159);
  betaRight = -asin(deltaZ/rightLength)*(180/3.14159);

  XservoR.writeMicroseconds(alphaRight*microsecondsPerDegree + 1500);
  ZservoR.writeMicroseconds(betaRight*microsecondsPerDegree + 1500);
  delay(2);
  
//  Serial.println("XSERVOR");
//  Serial.println(XservoR.read());
//  Serial.println("ZSERVOR");
//  Serial.println(ZservoR.read());
}
