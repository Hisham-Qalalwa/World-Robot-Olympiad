 
#include"mode.h"
#include"Pins.h"
#include"RGBAndMode.h"
#include"Ultrasonic.h"
#include"DriveCar.h"
#include"Rotation.h"

#include"setSteering.h"
#include"SerialComm.h"
#include"level1.h"
#include"level2.h"



unsigned long displayTime = 0;

void setup() {

  Serial.begin(115200);
  initRGBAndMode();
  led(RED);
  initCar();
  init_mpu();
  //initBT();
  initUltrasonic();

  initSteering();
  delay(500);
  readUltrasonic();
  initLevel1();
  motion_mode = STANDBY;
  checkLevel();
  driveTime = millis();
  
}
        
void loop() {

  
   //readSerial();
 // readUltrasonic();
  // readDistance(LEFT_RIGHT); 
 //debugUltrasonic();
  //Serial.print("steps to turn="); Serial.println((1000/46)*(initF-15));  
  //if((millis()-yawReadTime)>=READ_ANGLE_TIME){readAngle(); yawReadTime=millis();}
 //readAngle();
  //debugRotation();
  // readBT();
  checkStartButton();
  
  if (start) {

    if (level) {
 
      readDistance(readDistanseFrom);
      //ebugUltrasonic();
      doLevel2();
    }
    else {
    //  if((millis()-yawReadTime)>=READ_ANGLE_TIME){readAngle(); yawReadTime=millis();}
  
    readAngle(); 
      readUltrasonic();
      doLevel1();
    }
  }
  else {

     
    if (level) {
    //  Serial.println("LEVEL2");   readDistanseFrom
      readDistance(ALL);
  //  debugUltrasonicFrom(readDistanseFrom);
      initLevel2(); yawReadTime=millis();

      
    } else {
     // Serial.println("LEVEL1");
      readUltrasonic();
      initLevel1(); yawReadTime=millis();
    }
  }

  //if(millis()-displayTime>200){debugUltrasonic();/* debugLevel1(); debugRotation();*/ displayTime=millis();}

}
