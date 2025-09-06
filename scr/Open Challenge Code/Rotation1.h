#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long mpu_timer = 0;
int initAngle,currentAngle,angle0;
void init_mpu(){   
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ ESP.restart();} // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
}
void readAngle(){
  mpu.update();
  currentAngle=(int)mpu.getAngleZ();
  }
void debugRotation(){Serial.print("Angle= "); Serial.println(currentAngle);}
