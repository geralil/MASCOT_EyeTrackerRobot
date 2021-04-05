
void ReadAnalog()
{
  
  Analog_X = map(Xbox.getAnalogHat(LeftHatX),-32767,32767,0,1023);
  Analog_Y = map(Xbox.getAnalogHat(LeftHatY),-32767,32767,0,1023);
  Analog_Z = map(Xbox.getAnalogHat(RightHatY),-32767,32767,0,1023);
  
  

  //if the value is 25 "value away" from the average (midpoint), we allow the update of the speed
  //This is a sort of a filter for the inaccuracy of the reading
  if(abs(Analog_X-Analog_X_AVG)>Lim && abs(stepperX.currentPosition())<30000 ) 
  {
    stepperX.move(5*((Analog_X)-Analog_X_AVG));  
  }
  else
  {    
    stepperX.move(0);
  }
  //----------------------------------------------------------------------------  
  if(abs(Analog_Y-Analog_Y_AVG)>Lim) 
  { 
  stepperY.move(5*(Analog_Y-Analog_Y_AVG));  
  }
  else
  {
    stepperY.move(0);
  }
  //---------------------------------------------------------------------------
  if(abs(Analog_Z-Analog_R_AVG)>Lim) 
  {
    stepperZ.move(5*(Analog_Z-Analog_R_AVG));  
  }
  else
  {
    stepperZ.move(0);
  }
  //----------------------------------------------------------------------------  

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
  for(int i = 0; i<50; i++)
  {
    Usb.Task();
//     tempX += analogRead(Analog_X_pin);
tempX += map(Xbox.getAnalogHat(LeftHatX),-32767,32767,0,1023);
     
     delay(10); //allowing a little time between two readings
//     tempY += analogRead(Analog_Y_pin);      
tempY += map(Xbox.getAnalogHat(LeftHatY),-32767,32767,0,1023);   
     delay(10);
tempR += map(Xbox.getAnalogHat(RightHatX),-32767,32767,0,1023);  
     delay(10);
  }
  //----------------------------------------------------------------------------  
  Analog_X_AVG = tempX/50; 
  Analog_Y_AVG = tempY/50; 
  Analog_R_AVG = tempR/50; 
  //----------------------------------------------------------------------------  
  Serial.print("AVG_X: ");
  Serial.println(Analog_X_AVG);
  Serial.print("AVG_Y: ");
  Serial.println(Analog_Y_AVG);
  Serial.print("AVG_R: ");
  Serial.println(Analog_R_AVG);
  Serial.println("Calibration finished");  
}
