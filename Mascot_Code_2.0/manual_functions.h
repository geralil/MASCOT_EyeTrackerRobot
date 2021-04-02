
//int getMove()
//{
//  xStick=Xbox.getAnalogHat(LeftHatX);
//  zStick=Xbox.getAnalogHat(LeftHatY);
//  joystick=zStick;
//  joystick2=xStick;
//  LxScreen = map(joystick,-32767,32767,-1000,1000);
//  LzScreen = map(joystick2,-32767,32767,-1000,1000);
//
//  dir=LxScreen;
//
//
//  limit=digitalRead(7);
//  if(dir<-100 && limit==HIGH && Coordinates[xsteps]<45800) // Move left
//  {
//    del=(1100-(-1*(LxScreen-100)))/2;
//    digitalWrite(11,LOW);
//    digitalWrite(8,HIGH);
//    delayMicroseconds((int)del);
//    digitalWrite(8,LOW);
//    delayMicroseconds((int)del);
//    Coordinates[xsteps]++;
//  }
// 
//  
//  if(dir>100 && limit==HIGH && Coordinates[xsteps]>0) // Move Right
//  {
//    del=(1100-(LxScreen+100))/2;
//    digitalWrite(11,HIGH);
//    digitalWrite(8,HIGH);
//    delayMicroseconds((int)del);
//    digitalWrite(8,LOW);
//    delayMicroseconds((int)del);
//    Coordinates[xsteps]--;
//    
//  }
//
//  if(dir2<-100 && limit==HIGH) // Move left
//  {
//    del=(1100-(-1*(LzScreen-100)))/2;
//    digitalWrite(12,LOW);
//    digitalWrite(9,HIGH);
//    delayMicroseconds((int)del);
//    digitalWrite(9,LOW);
//    delayMicroseconds((int)del);
//    Coordinates[zsteps]--;
//  }
//  
//  if(dir2>100 && limit==HIGH) // Move Right
//  {
//    del=(1100-(LzScreen+100))/2;
//    digitalWrite(12,HIGH);
//    digitalWrite(9,HIGH);
//    delayMicroseconds((int)del);
//    digitalWrite(9,LOW);
//    delayMicroseconds((int)del);
//    Coordinates[zsteps]++;
//  }   
//}

