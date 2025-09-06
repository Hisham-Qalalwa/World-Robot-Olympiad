#include <Arduino.h>
#include <TinyMPU6050.h>

/*
 *  Constructing MPU-6050
 */
MPU6050 mpu (Wire);


unsigned long mpu_timer = 0;
int initAngle,currentAngle,angle0;

void init_mpu(){   
    // Initialization
    mpu.Initialize();

  
  
  Serial.println("=====================================");
  Serial.println("Starting calibration...");
  mpu.Calibrate();
  Serial.println("Calibration complete!");
  Serial.println("Offsets:");
  Serial.print("GyroX Offset = ");
  Serial.println(mpu.GetGyroXOffset());
  Serial.print("GyroY Offset = ");
  Serial.println(mpu.GetGyroYOffset());
  Serial.print("GyroZ Offset = ");
  Serial.println(mpu.GetGyroZOffset());
}
void readAngle(){
   mpu.Execute();
  currentAngle=(int)mpu.GetAngZ();
  }
void debugRotation(){Serial.print("Angle= "); Serial.println(currentAngle);}
