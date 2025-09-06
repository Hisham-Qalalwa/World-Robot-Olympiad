

#include <Wire.h>

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;
unsigned long mpu_timer = 0;
unsigned long readangleTime;
int initAngle,currentAngle,angle0;
 float yaw ;
 float nowAngle,prevAngle,cAngle;


void init_mpu(){  
  readangleTime=0; 
  Serial.println();
  Serial.println("init BNO080 Read Yaw angle ");
delay(100);

  

  Wire.begin();


  if (myIMU.begin() == false)
  {
    Serial.println(F("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing..."));
    while (1)
      ;
  }

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableRotationVector(50); //Send data update every 50ms

  Serial.println(F("Rotation vector enabled"));
 // Serial.println(F("Output in form roll, pitch, yaw"));
      if (myIMU.dataAvailable() == true)
  {
    Serial.println("First read");
     yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees
     

 
  }
   nowAngle=yaw; 
  prevAngle=nowAngle;
  currentAngle=0;
 cAngle=0;
}



void readAngle(){
 // delay(20);
  if(millis()-readangleTime>=65){readangleTime=millis();
    if (myIMU.dataAvailable() == true)
  {

     yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees
     

 
  }
  nowAngle=yaw;
    float deltaAngle;
  //Serial.print("Yaw Angle= "); Serial.print(yaw);
  if (nowAngle<0 && prevAngle>0){// 
        if(nowAngle< -160 && prevAngle>160){/* Serial.print("   +    : ");  */  deltaAngle=(180.0- prevAngle) + (180+ nowAngle); }
        else if(nowAngle>-20 && prevAngle<20){/* Serial.print("   -    : ");  */  deltaAngle= nowAngle-prevAngle;}
   
    }
  else if (nowAngle>0 && prevAngle<0){
        if(nowAngle>160 && prevAngle<-160){/*Serial.print("   -    : "); */ deltaAngle=((180+prevAngle)+(180-nowAngle))*-1;}
        if(nowAngle<20 && prevAngle>-20){/*Serial.print("   +    : ");*/ deltaAngle =nowAngle- prevAngle;}
    
   }
  else if (nowAngle>prevAngle){/*Serial.print("   +    :");*/ deltaAngle= nowAngle- prevAngle;}
  else if (nowAngle<prevAngle){/*Serial.print("   -   :"); */deltaAngle= nowAngle- prevAngle;}
  
  else{/*Serial.println("   ==   ");*/deltaAngle=0.0;}
  //Serial.println(deltaAngle);
  cAngle=cAngle+deltaAngle;
  currentAngle=(int)cAngle;
//  Serial.print("Robot Angle= "); Serial.println(currentAngle);
 prevAngle=nowAngle;
}
}
void debugRotation(){Serial.print("Yaw Angle= "); Serial.println(currentAngle);}
