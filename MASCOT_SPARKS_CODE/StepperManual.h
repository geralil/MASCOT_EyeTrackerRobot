/*
Read Analog function takes the XBOX analog stick inputs, subtracts the average of 
the XBOX analog zero positions to reduce noise and writes to .move() function
in Accelstepper. 
*/
void ReadAnalog()
{
  int Lim = 75; // The difference between stick reading and average value should be this
                // for any movement of steppers

	Analog_X = map(Xbox.getAnalogHat(LeftHatX), -32767, 32767, 0, 1023);
	Analog_Y = map(Xbox.getAnalogHat(LeftHatY), -32767, 32767, 0, 1023);
	Analog_Z = map(Xbox.getAnalogHat(RightHatY), -32767, 32767, 0, 1023);

	//if the value is 25 "value away" from the average (midpoint), we allow the update of the speed
	//This is a sort of a filter for the inaccuracy of the reading
	if (abs(Analog_X - Analog_X_AVG) > Lim && abs(stepperX.currentPosition()) < 30000)
	{
		stepperX.move(((Analog_X)-Analog_X_AVG));
	}
	else
	{
		stepperX.move(0);
	}
	//----------------------------------------------------------------------------  
	if (abs(Analog_Y - Analog_Y_AVG) > Lim)
	{
		stepperY.move((Analog_Y - Analog_Y_AVG));
	}
	else
	{
		stepperY.move(0);
	}
	//---------------------------------------------------------------------------
	if (abs(Analog_Z - Analog_R_AVG) > Lim)
	{
		stepperZ.move((Analog_Z - Analog_R_AVG));
	}
	else
	{
		stepperZ.move(0);
	}
	//----------------------------------------------------------------------------  

}
