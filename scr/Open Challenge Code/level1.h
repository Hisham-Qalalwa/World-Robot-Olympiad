unsigned long runTime=0;
unsigned long encoderCounterr;
bool isStarted;
bool  oncetime=true;
void debugLevel1(){
  Serial.print("encoder_count: "); Serial.print( encoder_count);
  Serial.print("rounCounter: "); Serial.print( rounCounter);
  Serial.print("setpoint: "); Serial.print( setpoint);
  Serial.println();
  }
void initLevel1(){
  complementaryMode=false;
   isStarted=false;
   initAngle=currentAngle;
   motion_mode=STANDBY;
   initR= rangeR;
   initL= rangeL;
   initF= rangeF;
   readAngle();
   //currentAngle=(int)mpu.getAngleZ();
   rounCounter=0;
   rotionDirection=UNKNOWN_DIRECTION;
   setpoint = rangeR;
  }
void doForwardWithEncoderSteps(int steps){
  if(encoder_count>steps){motion_mode=FORWARD;   Serial.print("setpoint stepsssssssss is : "); Serial.print( setpoint);  setpoint=setpoint-3;}
  else{
    if(rotionDirection==ANTI_CLOCKWIZE){ setpoint = rangeR;  }
    else if(rotionDirection==  CLOCKWIZE ){  setpoint = rangeL; }
    steering.write(steeringPID);forward(SLOW_SPEED4);}
  } 
void doForwardWithEncoder(){
  if(encoder_count>500){motion_mode=FORWARD_ENCODER;   Serial.print("setpoint after Stright incoder is : "); Serial.print( setpoint); }
  else{ steering.write(steeringPID);/* motion_mode=STOP;*/ forward(SPEED_L1);}
  } 
//*****************************
void doTurnEncoder(){
 
    if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+35; setSteering(steeringAngle);}
    else if (rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-43; setSteering(steeringAngle);}
    if(abs(currentAngle-initAngle)>=80){ steering.write(STRAIGHT_STEERING); rounCounter++; 
  
      if(rotionDirection==ANTI_CLOCKWIZE || rotionDirection==UNKNOWN_DIRECTION){ setpoint = rangeR;  }
      else if(rotionDirection==  CLOCKWIZE ){  setpoint = rangeL; }
          encoder_count=0; motion_mode=FORWARD_ENCODER; }
    
    
  }
//*****************************
void doTurn(){
     if(oncetime){  encoder_count=0;    while(encoder_count<100){ if(rotionDirection==ANTI_CLOCKWIZE){ setpoint = rangeR; } else{ setpoint = rangeL; } forward(SPEED2);  steering.write(steeringPID);}  encoder_count=0;  oncetime=false;}
  
    
    forward(SPEED2);
    if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING+35; setSteering(steeringAngle);}
    else if (rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING-43; setSteering(steeringAngle);}
    if(abs(currentAngle-initAngle)>=65 &&  encoder_count>700){stop(); steering.write(STRAIGHT_STEERING); delay(100);rounCounter++; 
       forward(SPEED_L1);
      if(rotionDirection==ANTI_CLOCKWIZE || rotionDirection==UNKNOWN_DIRECTION){ readUltrasonic(); setpoint = rangeR-4;  }
      else if(rotionDirection==  CLOCKWIZE ){  readUltrasonic(); setpoint = rangeL-4; }
      
           Serial.print("Encoder="); Serial.println(encoder_count);
           Serial.print("initAngle="); Serial.println(initAngle);
           Serial.print("Setpoint in turnr is : "); Serial.println(setpoint);
           Serial.print("currentAngle="); Serial.println(currentAngle);
           encoder_count=0;  motion_mode=FORWARD_STEPS;//LLLLLLLLLLLLLLLLLLLLL
          }
    
    
  }
void doPreFinal(){                       
  
   if((rotionDirection== CLOCKWIZE )&& (rangeF<(initL+25) && rangeF>1) &&  (rangeR>120 ||  rangeR==0)){initAngle=currentAngle; motion_mode=TURNE;}
    else if((rotionDirection==ANTI_CLOCKWIZE )&& (rangeF<(initR+25) && rangeF>1) &&  (rangeL>120 ||  rangeL==0)){initAngle=currentAngle; motion_mode=TURNE;}
    else{ forward(SPEED_L1);steering.write(steeringPID); } // go straight
  }
void doParking(){
  if(rangeF <146 && rangeF!=0 && encoder_count>950 ){Serial.print("Encoder"); Serial.println(encoder_count); motion_mode=STOP;}
  else{forward(SPEED_L1);steering.write(steeringPID);}
  } 
void doForward(){
  oncetime=true;
  //SLOW_SPEED2
  if(rotionDirection==UNKNOWN_DIRECTION){
    // if direction not set
    forward(SLOW_SPEED4);
    if(rangeF<35){delay(100);}
    else if(rangeF<55){forward(SLOW_SPEED2); }

    
    if((rangeR>120  &&   ((rangeR-rangeL)>80))){ rotionDirection=CLOCKWIZE;  }
    else if((rangeL>120  && ((rangeL-rangeR)>80)) /* || rangeL==0*/){rotionDirection=ANTI_CLOCKWIZE;}
    }
  else{
 // if(rounCounter==11){motion_mode=PRE_FINAL;}
  if(rounCounter==12){motion_mode=PARKING;}                                                               /*  ,prevRangeL*/
  else{
    if((rotionDirection== CLOCKWIZE )&& (rangeF<80   && rangeF>1) &&  (rangeR>120  ||  rangeR==0 )){
      
      initAngle=currentAngle; motion_mode=TURNE;
      steeringAngle=STRAIGHT_STEERING+35; setSteering(steeringAngle);
      encoder_count=0; 
     // if(rounCounter==3){motion_mode=STOP;}
       Serial.print("Direction  = "); Serial.println(rotionDirection );
       
      Serial.print("Set point = "); Serial.println(setpoint );
      Serial.print("R"); Serial.println(rangeR );
      Serial.print("Encoder="); Serial.println(encoder_count );
      }
    else if((rotionDirection==ANTI_CLOCKWIZE )&& (rangeF<95 &&  rangeF>1) &&  ((rangeL>105)||  ( rangeL==0 )) || (rangeF<50 && rangeF>3)){
      initAngle=currentAngle;  motion_mode=TURNE;
      encoder_count=0; 
    //  if(rounCounter==3){motion_mode=STOP;}
      
     Serial.print("Direction  = "); Serial.println(rotionDirection );
    Serial.print("Set point = "); Serial.println(setpoint );
    Serial.print("L"); Serial.println(rangeL );
    Serial.print("Encoder="); Serial.println(encoder_count );
    }
    else{ forward(SPEED_L1);steering.write(steeringPID); debugUltrasonic();} // go straight
    }
  } 

}
//the main function of level 1
void doLevel1(){
  switch (motion_mode) 
  {
  case STANDBY:
  break;
  case FORWARD:
    if(!isStarted){isStarted=true; runTime=millis();}
    doForward();    
  break;
  case FORWARD_STEPS:
       doForwardWithEncoderSteps(300);
  break;
  case FORWARD_ENCODER:
    doForwardWithEncoder();
  break;
  case PRE_FINAL:
    doPreFinal();
  break;
  case TURNE:
    doTurn();
  break;
  case PARKING:
    doParking();
  break;
  case STOP:
    stop();
    isStarted=false ; Serial.print("Time is : ");Serial.println((millis()-runTime)/1000);
    ESP.restart();
  break;
  }
  }
