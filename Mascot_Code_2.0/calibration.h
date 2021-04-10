void servoCalibration(){

  DotPos[0] = 0;
  DotPos[1] = 0;
   x = (0.5*IPD); //x = distance from left side of screen to eye
   z = (eyeZlocation); //distance between top of screen and eye  //eyeZlocation is the measured distance between chin laser and eye
//   alphaLeft = atan(x/deltaY)*(180/3.14159);
//   betaLeft = atan(z/deltaY)*(180/3.14159);
//   alphaRight = atan(x/deltaY)*(180/3.14159);
//   betaRight = atan(z/deltaY)*(180/3.14159);

  //Step 1: Point left eye and right eye at 90 position
    i=0;
    while(i==0) //Waits for B to be pressed on Xbox controller
    {
      Usb.Task();
      if(Xbox.getButtonClick(B))
      {
        i=1;
      }
      GetPos();
      leftEye();
//      Serial.println("Xservo");
//      Serial.println(XservoL.read());
      //Serial.println("Zservo");
      //Serial.println(ZservoL.read());
//      Serial.println("DotposX");
//      Serial.println(DotPos[0]);
//      Serial.println("DotposZ");
//      Serial.println(DotPos[1]);
    }

left90xhome = XservoL.read();  //servo x angle for left eye center
left90xmicro = XservoL.readMicroseconds();
left90zhome = ZservoL.read();  //servo z angle for left eye center
left90zmicro = XservoL.readMicroseconds();

  DotPos[0] = 0;
  DotPos[1] = 0;

  Serial.println("Left Length ");
  Serial.println(leftLength);
  Serial.println("deltaX ");
  Serial.println(deltaX);
  Serial.println("Delta Y ");
  Serial.println(deltaY);
  Serial.println("Delta z");
  Serial.println(deltaZ);
  Serial.println("left X home");
  Serial.println(left90xhome);
  Serial.println("left z home");
  Serial.println(left90zmicro);
  Serial.println(alphaLeft);
  Serial.println(betaLeft);

delay(1000);

    i=0;
    while(i==0) //Waits for B to be pressed on Xbox controller
    {
      Usb.Task();
      if(Xbox.getButtonClick(B))
      {
        i=1;
      }
      GetPos();
      rightEye();      
//      Serial.println("Xservo");
//      Serial.println(XservoR.read());
//      Serial.println("Zservo");
//      Serial.println(ZservoR.read());
//      Serial.println("DotposX");
//      Serial.println(DotPos[0]);
//      Serial.println("DotposZ");
//      Serial.println(DotPos[1]);
    }
right90xhome = XservoR.read();  //servo x angle for right eye center
right90xmicro = XservoL.readMicroseconds();
right90zhome = ZservoR.read();  //servo z angle for right eye center 
right90zmicro = XservoL.readMicroseconds();

  Serial.println("Right Length ");
  Serial.println(rightLength);
  Serial.println("deltaX ");
  Serial.println(deltaX);
  Serial.println("Delta Y ");
  Serial.println(deltaY);
  Serial.println("Delta z");
  Serial.println(deltaZ);
  Serial.println("right X home");
  Serial.println(right90xhome);
  Serial.println("right z home");
  Serial.println(right90zmicro);
  Serial.println(alphaRight);
  Serial.println(betaRight);
    DotPos[0] = 0;
  DotPos[1] = 0;

  //Step 2: Point left eye and right eye at the chin laser  
    i=0;
    while(i==0) //Waits for B to be pressed on Xbox controller
    {
      Usb.Task();
      if(Xbox.getButtonClick(B))
      {
        i=1;
      }
      GetPos();
      leftEye();
//      Serial.println("Xservo");
//      Serial.println(XservoL.read());
//      Serial.println("Zservo");
//      Serial.println(ZservoL.read());
    }

//leftXhome = XservoL.read();  //servo x angle for left eye center
//leftZhome = ZservoL.read();  //servo z angle for left eye center

  offsetLx = atan((0-DotPos[0])/deltaY);
  offsetLz = atan((0-DotPos[1])/deltaY);
  
  DotPos[0] = 0;
  DotPos[1] = 0;

  Serial.println("Left Length ");
  Serial.println(leftLength);
  Serial.println("deltaX ");
  Serial.println(deltaX);
  Serial.println("Delta Y ");
  Serial.println(deltaY);
  Serial.println("Delta z");
  Serial.println(deltaZ);
  Serial.println("left X home");
  Serial.println(leftXhome);
  Serial.println("left z home");
  Serial.println(leftZhome);
  Serial.println(alphaLeft);
  Serial.println(betaLeft);

delay(1000);

    i=0;
    while(i==0) //Waits for B to be pressed on Xbox controller
    {
      Usb.Task();
      if(Xbox.getButtonClick(B))
      {
        i=1;
      }
      GetPos();
      rightEye();      
//      Serial.println("Xservo");
//      Serial.println(XservoR.read());
//      Serial.println("Zservo");
//      Serial.println(ZservoR.read());
    }
//rightXhome = XservoR.read();  //servo x angle for right eye center
//rightZhome = ZservoR.read();  //servo z angle for right eye center 

  offsetRx = atan((0-DotPos[0])/deltaY);
  offsetRz = atan((0-DotPos[1])/deltaY);

  Serial.println("Right Length ");
  Serial.println(rightLength);
  Serial.println("deltaX ");
  Serial.println(deltaX);
  Serial.println("Delta Y ");
  Serial.println(deltaY);
  Serial.println("Delta z");
  Serial.println(deltaZ);
  Serial.println("right X home");
  Serial.println(rightXhome);
  Serial.println("right z home");
  Serial.println(rightZhome);
  Serial.println(alphaRight);
  Serial.println(betaRight);
    DotPos[0] = 0;
  DotPos[1] = 0;
  
  DotPos[xpos]=screenWidth/2;
  DotPos[zpos]=screenHeight/2;
}

void stepperCalibration()
{
  //Step 1: Home the rig.
  zeroStepper(ydir,ypulse,yEnd);
  stepperY.setCurrentPosition(500);
  zeroStepper(zdir,zpulse,zEnd);
  stepperZ.setCurrentPosition(500);
  zeroStepper(xdir,xpulse,xEnd);
  stepperX.setCurrentPosition(-500);
  
  //Step 2: Point chin laser at the bottom left corner of the screen @ 65cm.
  moveStepper_relative(zdir,zpulse,8335,HIGH);
  moveStepper_relative(ydir,ypulse,(10188+4*StepsPerMM),HIGH); // Last input parameter: Steps from home position to 65cm.

Usb.Task();

i=0;
while(i==0) //Waits for B to be pressed on Xbox controller
{
  
  Usb.Task();
  if(Xbox.getButtonClick(B))
  {
    i=1; 
  }
  ReadAnalog(); 
  stepperX.run(); 
  stepperZ.run();
}

bottomScreen=abs(stepperZ.currentPosition());
leftScreen=abs(stepperX.currentPosition());
Serial.println("Bottom Left Corner");
Serial.print(bottomScreen);
Serial.print("\t");
Serial.println(leftScreen);

delay(1000);


i=0;
while(i==0) //Waits for B to be pressed on Xbox controller
{
  
  Usb.Task();
  if(Xbox.getButtonClick(B))
  {
    i=1;
  }
  ReadAnalog(); 
  stepperX.run(); 
  stepperZ.run();
}

topScreen=abs(stepperZ.currentPosition());
rightScreen=abs(stepperX.currentPosition());
Serial.println("Top Right Corner");
Serial.print(topScreen);
Serial.print("\t");
Serial.println(rightScreen);


moveStepper_relative(xdir,xpulse,(rightScreen-leftScreen)/2,HIGH);
moveStepper_relative(zdir,zpulse,(topScreen-bottomScreen)/2,LOW);

stepperX.setCurrentPosition(0);
stepperZ.setCurrentPosition(0);
stepperY.setCurrentPosition(10688);


Serial.println("Current Position");
Serial.print(stepperZ.currentPosition());
Serial.print("\t");
Serial.println(abs(stepperX.currentPosition()));

  DotPos[xpos]=screenWidth/2;
  DotPos[zpos]=screenHeight/2;
}


void findCoordinates() // This function will be used whenever the Tobii Calibration coordinates need to be reset
{
   int j=0;
   while(j==0)
   {
    while(i==0&&j==0)
    {
      Usb.Task();
       GetPos();
       parallax();
       i=0;
      if(Xbox.getButtonClick(B))
      {
        i=1;
      }
      if(Xbox.getButtonClick(X))
      {
        j=1;
      }
     
    }
/* The X and Z coordinates will be displayed on the serial monitor one at a time. 
 *  These must be writeen down and entered into the calibrationCoords array. 
 */
Serial.print(DotPos[xpos]);
Serial.print("\t");
Serial.println(DotPos[zpos]);
i=0;
   }
   state=MenuMode;
}

void setPos() // This function will steer the gaze of the artificial eyes at the calibration points during the Tobii Calibration.
{
  for(int j=0;j<4;j++)
  {
    DotPos[0]=calibrationCoords[0][j]; // X Coordinate
    DotPos[1]=calibrationCoords[1][j]; // Z Coordinate
    parallax();
    i=0;
  while(i==0) // Waits for the user to advance the index by clicking X
    {
      Usb.Task();
       i=0;
      if(Xbox.getButtonClick(X))
      {
        i=1;
      } 
    }
    i=0;
 }
 state=MenuMode;
}
