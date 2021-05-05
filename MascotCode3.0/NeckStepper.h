/*
 * Maps the input to a new output range.
 */
float mapf(float x, float in_min, float in_max, float out_min, float out_max) 
{
     float result;
     result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
     return result;
}

long lastjoystick = micros();

/*
 * Function that takes input from the XBOX controller and converts into NECK stepper input
 * Function provides analog control of the neck steppers.
 */
void ReadAnalogNeck()
{
  if ((micros() - lastjoystick) < 200000)
  {
    return;
  }

  lastjoystick = micros();

  // mapping the XBOX analog inputs between -1 and 1
  Analog_XN = mapf(float(Xbox.getAnalogHat(LeftHatX)),-32767.0,32767.0,-1.0,1.0);
  Analog_YN = mapf(float(Xbox.getAnalogHat(LeftHatY)),-32767.0,32767.0,-1.0,1.0);
  Analog_Yaw = mapf(Xbox.getAnalogHat(RightHatX), -32767, 32767, -1, 1);
  
  bool shouldupdatepos = false ;

  // finding the difference between the analog input and the average analog calibration value.
  float deltax = Analog_XN-Analog_XN_AVG;
  float deltay = Analog_YN-Analog_YN_AVG;
  float deltayaw = Analog_Yaw-Analog_RN_AVG;

  /*
   * if the delta value determined from analog input is greater than
   * the limiting value for stepper movement, then the boolean value is turned true.
   */
  if (abs(deltax) > limN)
  {
    desiredneckx += gainx * deltax;
    shouldupdatepos = true;
  }
  if (abs(deltay) > limN)
  {
    desirednecky += gainy * deltay;
    shouldupdatepos = true;
  }
  if (abs(deltayaw) > limN)
  {
    desiredneckyaw += gainyaw * deltayaw;
    shouldupdatepos = true;
  }

  /*
   * If the boolean value is true, then the stepper positions are updated.
   */
  if (shouldupdatepos == true) //If joystick position isnt at middle position
  {
    PhiS = atan2(deltay,deltax) - M_PI_2;
    PhiR = (3.1459/6) * sqrt(sq(deltay) + sq(deltax));

    Serial.print("Phi S   ");
    Serial.println(PhiS);
    Serial.print("Phi R   ");
    Serial.println(PhiR);

    BLA::Matrix<6> xi_01 = {0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    BLA::Matrix<6> xi_12 = {0.0, 0.0, 1.0, 0.0, PhiR/Lspring, 0.0};
    BLA::Matrix<6> xi_23 = {0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    //g00 
    BLA::Matrix<4,4> g01 = expmxi(xi_01*PhiS);
    BLA::Matrix<4,4> g02 = expmxi(xi_12*Lspring);
    BLA::Matrix<4,4> g03 = expmxi(xi_23*(-PhiS));

    BLA::Matrix<4,4> gg03 = g01*g02*g03;

    BLA::Matrix<3> EulerVector = EVector(gg03);
    BLA::Matrix<3> TranslationVector = TVector(gg03);
    
    Serial.println("GG03 Euler Vector");
    Serial<<EulerVector;
    Serial.println("   ");

    Serial.println("GG03 Translation Vector");
    Serial<<TranslationVector;
    Serial.println("   ");

    Serial.println(deltayaw);
    

    BLA::Matrix<4,4> g_O_B1 = {1.0, 0.0, 0.0, 0.06*cos(0*2*3.14159/3),
                             0.0, 1.0, 0.0, 0.06*sin(0*2*3.14159/3),
                             0.0, 0.0, 1.0, 0.0,
                             0.0, 0.0, 0.0, 1.0};
    BLA::Matrix<4,4> g_O_B2 = {1.0, 0.0, 0.0, 0.06*cos(2*3.14159/3),
                             0.0, 1.0, 0.0, 0.06*sin(2*3.14159/3),
                             0.0, 0.0, 1.0, 0.0,
                             0.0, 0.0, 0.0, 1.0};
    BLA::Matrix<4,4> g_O_B3 = {1.0, 0.0, 0.0, 0.06*cos(2*2*3.14159/3),
                             0.0, 1.0, 0.0, 0.06*sin(2*2*3.14159/3),
                             0.0, 0.0, 1.0, 0.0,
                             0.0, 0.0, 0.0, 1.0};
    BLA::Matrix<4,4> g_Pc_P1 = {1.0, 0.0, 0.0, 0.06*cos(0*2*3.14159/3),
                              0.0, 1.0, 0.0, 0.06*sin(0*2*3.14159/3),
                              0.0, 0.0, 1.0, 0.0,
                              0.0, 0.0, 0.0, 1.0};
    BLA::Matrix<4,4> g_Pc_P2 = {1.0, 0.0, 0.0, 0.06*cos(2*3.14159/3),
                              0.0, 1.0, 0.0, 0.06*sin(2*3.14159/3),
                              0.0, 0.0, 1.0, 0.0,
                              0.0, 0.0, 0.0, 1.0};
    BLA::Matrix<4,4> g_Pc_P3 = {1.0, 0.0, 0.0, 0.06*cos(2*2*3.14159/3),
                              0.0, 1.0, 0.0, 0.06*sin(2*2*3.14159/3),
                              0.0, 0.0, 1.0, 0.0,
                              0.0, 0.0, 0.0, 1.0};

    BLA::Matrix<4,4> g_B1_O = g_O_B1.Inverse();
    BLA::Matrix<4,4> g_B2_O = g_O_B2.Inverse();
    BLA::Matrix<4,4> g_B3_O = g_O_B3.Inverse();

    BLA::Matrix<4,4> g_B1_P1 = g_B1_O*gg03*g_Pc_P1;
    BLA::Matrix<4,4> g_B2_P2 = g_B2_O*gg03*g_Pc_P2;
    BLA::Matrix<4,4> g_B3_P3 = g_B3_O*gg03*g_Pc_P3;

    BLA::Matrix<4,4> g_O_P1 = gg03*g_Pc_P1;
    BLA::Matrix<4,4> g_O_P2 = gg03*g_Pc_P2;
    BLA::Matrix<4,4> g_O_P3 = gg03*g_Pc_P3;

    CableOneLength = sqrt(sq(g_B1_P1(0,3)) + sq(g_B1_P1(1,3)) + sq(g_B1_P1(2,3)));
    CableTwoLength = sqrt(sq(g_B2_P2(0,3)) + sq(g_B2_P2(1,3)) + sq(g_B2_P2(2,3)));
    CableThreeLength = sqrt(sq(g_B3_P3(0,3)) + sq(g_B3_P3(1,3)) + sq(g_B3_P3(2,3)));

    desiredstepperposOne = CableOneLength/mmperstep;                 //(laststepperposOne/lastCableLengthOne);
    desiredstepperposTwo = CableTwoLength/mmperstep;                 //(laststepperposTwo/lastCableLengthTwo); //mmperstep;
    desiredstepperposThree = CableThreeLength/mmperstep;             //(laststepperposThree/lastCableLengthThree);  //mmperstep;
  
    stepperOne.moveTo(desiredstepperposOne);
    //stepperOne.setSpeed(VelocityStepperOne);
    stepperTwo.moveTo(desiredstepperposTwo);
    //stepperTwo.setSpeed(VelocityStepperTwo);
    stepperThree.moveTo(desiredstepperposThree);
    //stepperThree.setSpeed(VelocityStepperThree); 
    NeckServo.write(83 + 30);
    
    laststepperposOne = stepperOne.currentPosition();
    laststepperposTwo = stepperTwo.currentPosition();
    laststepperposThree = stepperThree.currentPosition();
  } 
}
