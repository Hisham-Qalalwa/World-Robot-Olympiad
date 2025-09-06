

//volatile double last_time;

volatile int angPID;
volatile int outputa=0;
   //H/W timer defining (Pointer to the Structure)
double integrala, previousa = 0.0;

double kpa, kia, kda;
volatile double setpointa;

double pida(double error)
{
  double proportionala = error;
  integrala += error * dt;
  double derivativea = (error - previous) / dta;
  previous = error;
  double outputa = (kpa * proportionala) + (kia * integrala) + (kda * derivativea);
  return outputa;
}

void IRAM_ATTR setStiringang()
{
  
 // Serial.print("dt:"); Serial.println(dt);
 
  double actuala;
  if(rotionDirection==ANTI_CLOCKWIZE || rotionDirection==UNKNOWN_DIRECTION){actuala = (double)rangeR;}
  else if(rotionDirection==  CLOCKWIZE ){actuala = (double)rangeL;}
  if(actuala!=0.0){
  //Serial.print("dist f b="); Serial.println(actual);
  double errora= setpoint - actual;
  //Serial.print("setpoint="); Serial.println(setpoint);
  outputa =(int) pida(errora);
  output=constrain(outputa, -4200,4200);
  output=map(outputa,-4200,4200,-20,20);
  output=constrain(outputaa, -10,10);


  if(rotionDirection==ANTI_CLOCKWIZE || rotionDirection==UNKNOWN_DIRECTION){ 
      steeringPID=STRAIGHT_STEERING-output;}
  else if(rotionDirection==  CLOCKWIZE ){steeringPID=STRAIGHT_STEERING+output; }
}

}

void initAngle()
{ pidState=false;
  kpa = 280.0;
  kia = 0.0;
  kda = 23.0;
  dta=(double)INT_TIME/1000000;
  timer = timerBegin(0, 80, true);             // timer 3, prescalar: 80, UP counting
  timerAttachInterrupt(timer, &setStiring_, true);   // Attach interrupt
  timerAlarmWrite(timer, INT_TIME, true);     // Match value= 70000 for 70 msec. delay.
  timerAlarmEnable(timer);                 // Enable Timer with interrupt (Alarm Enable)
 
 // setpoint = 15.0;
  steering.write(STRAIGHT_STEERING);
 
}
////////////////////////////////////////////////////


void driveBetween(){
  // setStiring_();
  }
