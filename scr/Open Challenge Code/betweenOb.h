unsigned long yawReadTime, reverceTime;
int Factor;
int encodeSteps,siftDist;
int UTurnDirection;
int rotitionEncoder;
int preUTurneAngle, nextAngle;
#define UTURN_LEFT 1
#define UTURN_RIGHT 2
#define READ_ANGLE_TIME 55
String shift="";


//
// SHIFT==>>> SHIFT==>>> SHIFT==>>> SHIFT==>>> SHIFT==>>> SHIFT==>>> SHIFT==>>> SHIFT==>>> SHIFT==>>> SHIFT==>>> SHIFT==>>>    

void shifLeftCm(int C){
         
   switch(shift_Stages){ 
         case BRE_SHIFT:
         encodeSteps=(int)((-0.72*C*C)+(38*C)+77.7);
         shift_Stages=TURN1;
         encoder_count=0;
         break;
         case TURN1:
              
              if(steeringAngle!=STRAIGHT_STEERING-35){stop(); steeringAngle=STRAIGHT_STEERING-35;  turnSteering(STRAIGHT_STEERING,steeringAngle,5); } 
              
              if (encoder_count>=encodeSteps){stop(); shift_Stages=TURN2;/* stop();*/ encoder_count=0;  /*delay(50);*/ }
              else{ forward(SHIFT_SPEED);Serial.print ("encoder_count T1="); Serial.println(encoder_count);}
         break;

         case TURN2:
              forward(SHIFT_SPEED);
              if(steeringAngle!=STRAIGHT_STEERING+35){stop(); steeringAngle=STRAIGHT_STEERING+35;  turnSteering(STRAIGHT_STEERING-35,steeringAngle,10); }// setSteering(steeringAngle);
              if (encoder_count>=encodeSteps){stop();
                    shift_Stages=STRIGHT; /*stop();*/shift_Stages=FINISH;encoder_count=0; 
                    turnSteering(steeringAngle,STRAIGHT_STEERING,10);  /* delay(50);*/steeringAngle=STRAIGHT_STEERING;}
                     else{ forward(SHIFT_SPEED); Serial.print ("encoder_count T2="); Serial.println(encoder_count);}
         break;
         case FINISH:                                
           setSteering(STRAIGHT_STEERING);
        // motion_mode=STOP;
         break;  
   }
   

  }
//approximate function to shift right robot in Cm unit 
  void shifRightCm(int C){   
         
   switch(shift_Stages){
         case BRE_SHIFT:
         encodeSteps=(int)((-0.72*C*C)+(38*C)+77.7);
         shift_Stages=TURN1;
         encoder_count=0;
         break;
         case TURN1:
              
              if(steeringAngle!=STRAIGHT_STEERING+35){stop(); steeringAngle=STRAIGHT_STEERING+35;  turnSteering(STRAIGHT_STEERING,steeringAngle,5); } 
              
              if (encoder_count>=encodeSteps){stop(); shift_Stages=TURN2;/* stop();*/ encoder_count=0;  /*delay(50);*/ }
              else{ forward(SHIFT_SPEED);Serial.print ("encoder_count T1="); Serial.println(encoder_count);}
         break;

         case TURN2:
              forward(SHIFT_SPEED);
              if(steeringAngle!=STRAIGHT_STEERING-35){stop(); steeringAngle=STRAIGHT_STEERING-35;  turnSteering(STRAIGHT_STEERING+35,steeringAngle,10); }// setSteering(steeringAngle);
              if (encoder_count>=encodeSteps){stop();
                    shift_Stages=STRIGHT; /*stop();*/shift_Stages=FINISH;encoder_count=0; 
                    turnSteering(steeringAngle,STRAIGHT_STEERING,10);  /* delay(50);*/steeringAngle=STRAIGHT_STEERING;}
                     else{ forward(SHIFT_SPEED); Serial.print ("encoder_count T2="); Serial.println(encoder_count);}
         break;
         case FINISH:                                
           setSteering(STRAIGHT_STEERING);
       //  motion_mode=STOP;
         break;  
   }
   
  
  }
//approximate function to shift left robot in Cm unit 
void shifLeft(int C){
         
   switch(shift_Stages){
         case BRE_SHIFT:
         
         shift_Stages=TURN1;
         encoder_count=0;
         break;
         case TURN1:
               forward(SHIFT_SPEED);
              if(steeringAngle!=STRAIGHT_STEERING-35){steeringAngle=STRAIGHT_STEERING-35;  turnSteering(STRAIGHT_STEERING,steeringAngle,10); } 
             
              if (encoder_count>=c){shift_Stages=STRIGHT;/* stop();*/ encoder_count=0; /*delay(50);*/}
         break;
         case STRIGHT:
               //stop();
               turnSteering(steeringAngle,STRAIGHT_STEERING,8);  steeringAngle=STRAIGHT_STEERING;
              // setSteering(STRAIGHT_STEERING);
               shift_Stages=TURN2;
               encoder_count=0;
              // delay(20);
         break;
         case TURN2:
              forward(SHIFT_SPEED);
              if(steeringAngle!=STRAIGHT_STEERING+35){steeringAngle=STRAIGHT_STEERING+35;  turnSteering(STRAIGHT_STEERING,steeringAngle,10); }// setSteering(steeringAngle);
              if (encoder_count>=c){
                    shift_Stages=STRIGHT; /*stop();*/shift_Stages=FINISH;encoder_count=0; 
                    turnSteering(steeringAngle,STRAIGHT_STEERING,10);  /* delay(50);*/steeringAngle=STRAIGHT_STEERING;}
         break;
         case FINISH:                                
           setSteering(STRAIGHT_STEERING);
        // motion_mode=STOP;
         break;  
   }
   
  
  }
void shiftRight(int c){
  
   switch(shift_Stages){
         case BRE_SHIFT:
         
         shift_Stages=TURN1;
         encoder_count=0;
         break;
         case TURN1:
               forward(SLOW_SPEED2);
              if(steeringAngle!=STRAIGHT_STEERING+35){steeringAngle=STRAIGHT_STEERING+35; turnSteering(STRAIGHT_STEERING,steeringAngle,10); }
             
              if (encoder_count>=c){shift_Stages=STRIGHT;/*stop();delay(50);*/encoder_count=0; }
         break;
         case STRIGHT:
           //   stop();
              turnSteering(steeringAngle,STRAIGHT_STEERING,8);  steeringAngle=STRAIGHT_STEERING;//setSteering(STRAIGHT_STEERING); 
               shift_Stages=TURN2;
               encoder_count=0;
             //  delay(20);
         break;
         case TURN2:
              forward(SLOW_SPEED2);
              if(steeringAngle!= STRAIGHT_STEERING-35){steeringAngle=STRAIGHT_STEERING-35; turnSteering(STRAIGHT_STEERING,steeringAngle,10);} // setSteering(steeringAngle); 
              if (encoder_count>=c){shift_Stages=STRIGHT;/*stop();*/shift_Stages=FINISH;encoder_count=0;
              turnSteering(steeringAngle,STRAIGHT_STEERING,10);  /* delay(50);*/steeringAngle=STRAIGHT_STEERING;}
         break;
         case FINISH:
           setSteering(STRAIGHT_STEERING);
     //    motion_mode=STOP;
         break;  
   }
   
  
  }
// SHIFT==>>> SHIFT==>>> SHIFT==>>> SHIFT==>>> SHIFT==>>> SHIFT==>>> SHIFT==>>> SHIFT==>>> SHIFT==>>> SHIFT==>>> SHIFT==>>>
//


// move robote in side outer boarder FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
void fff(){
   complementaryMode=false;
    currentPosition='F'; setpoint=15;
   if(/*rangeL<120 &&*/ encoder_count<2000 && rotionDirection==ANTI_CLOCKWIZE){if(encoder_count<1700){forward(SLOW_SPEED2); } else {forward(SLOW_SPEED);}setSteering(steeringPID); }
   else if(/*rangeR<120 && */encoder_count<2000 && rotionDirection==CLOCKWIZE){if(encoder_count<1700){forward(SLOW_SPEED2); } else {forward(SLOW_SPEED);}setSteering(steeringPID); }
   else{ currentPosition='F'; move_Stage=STAGE1;  obstacle_Stage=FINAL; }}
// move robote in side inner boarder   NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNn
void nnn(){
 
  currentPosition='N';  //65 must change sensor
       if(rotionDirection==ANTI_CLOCKWIZE){setpoint=73; complementaryMode=true;}
       else if(rotionDirection==CLOCKWIZE){setpoint=73; complementaryMode=true;}
  if(/*rangeL<120 && */encoder_count<1800 && rotionDirection==ANTI_CLOCKWIZE){if(encoder_count<1600){forward(SLOW_SPEED2); } else {forward(SLOW_SPEED);}  setSteering(steeringPID); }
  else if(/*rangeR<120 && */encoder_count<1800 && rotionDirection==CLOCKWIZE){if(encoder_count<1600){forward(SLOW_SPEED2); } else {forward(SLOW_SPEED);} setSteering(steeringPID); }
  else{currentPosition='N'; move_Stage=STAGE1; obstacle_Stage=FINAL;  }
  }
//pre parking function in Head Area
void nnnH(){

      switch(move_Stage){
    case STAGE1:
         currentPosition='N';  //65 must change sensor
      complementaryMode=true;setpoint=73;
  if(/*rangeL<120 && */encoder_count<1600 && rotionDirection==ANTI_CLOCKWIZE){if(encoder_count<1450){forward(SLOW_SPEED2); } else {forward(SLOW_SPEED);}  setSteering(steeringPID); }
  else if(/*rangeR<120 && */encoder_count<1600 && rotionDirection==CLOCKWIZE){if(encoder_count<1450){forward(SLOW_SPEED2); } else {forward(SLOW_SPEED);} setSteering(steeringPID); }
  else{currentPosition='N'; move_Stage=STAGE2; stop(); }

    break;
    case STAGE2:


    
       move_Stage=STAGE3;
       readDistance(LEFT_RIGHT);
       Serial.print("Range L= ");  Serial.print(rangeL);  Serial.print("   Range R= ");  Serial.print(rangeR);
       //encoderStright=constrain(rangeR, 15, 26);   encoderStrightOb
        if(rotionDirection==ANTI_CLOCKWIZE){encoderStrightOb=constrain(rangeL, 5, 20); encoderStrightOb=map(encoderStrightOb,5,20,750,100);}
        else if(rotionDirection==CLOCKWIZE){encoderStrightOb=constrain(rangeR, 5, 20);  encoderStrightOb=map(encoderStrightOb,5,20,800,300);}
       Serial.print("   encoderStrightOb= ");  Serial.println(encoderStrightOb);
       tempAng=currentAngle;
       encoder_count=0;

    break;
        case STAGE3:
    //steeringAngle!=STRAIGHT_STEERING-37){steeringAngle=STRAIGHT_STEERING-37; turnSteering(STRAIGHT_STEERING,steeringAngle,10); 
       if(rotionDirection==ANTI_CLOCKWIZE && steeringAngle!=STRAIGHT_STEERING+37){steeringAngle=STRAIGHT_STEERING+37; turnSteering(STRAIGHT_STEERING,steeringAngle,10); /*turne right*/}
       else if(rotionDirection==CLOCKWIZE && steeringAngle!=STRAIGHT_STEERING-37){steeringAngle=STRAIGHT_STEERING-37; turnSteering(STRAIGHT_STEERING,steeringAngle,10);/*turne left*/}

       if (encoder_count<1100  && abs(currentAngle-tempAng)<80 ){forward(TURN_SPEED_L2);}
       else{ move_Stage=STAGE4; setSteering(STRAIGHT_STEERING);stop();delay(20); encoder_count=0;Serial.println("Finish final turne"); reverceTime=millis();}
    break;
    case STAGE4:
           digitalWrite(POS1,HIGH); digitalWrite(POS2,HIGH);
        if( readBackDistance(3)>=5 && encoder_count<1140 && (millis()-reverceTime)<1150 ){reverse(REVERSE_PARKING);    digitalWrite(POS1,HIGH);
                                                                                                                       digitalWrite(POS2,HIGH);}
        else {stop(); move_Stage=STAGE6; shift_Stages = BRE_SHIFT; 
        Serial.print("ENCODER COUNT = "); Serial.println(encoder_count);
        Serial.print("TIME = "); Serial.println(millis()-reverceTime);} 
       
        break;
    case STAGE5:  
                
        if(rotionDirection==ANTI_CLOCKWIZE){ shifLeftCm(6);}
        else if(rotionDirection==CLOCKWIZE){ shifRightCm(6);}
        
        if(shift_Stages ==  FINISH){stop(); move_Stage=STAGE6;}
    break;
    
    case STAGE6:

       Serial.println("Finish bre parking ");
        Parking_stages=CLEAR_SERIAL;
        obstacle_Stage=FINAL;
        motion_mode= PARKING;
      //  move_Stage=STAGE1;

    break;
    
      }  // shift_Stages = BRE_SHIFT;// shift_Stages =  FINISH;
  
  }

//pre parking function in Tail  Area
void nnnT(){

      
    switch(move_Stage){
    case STAGE1:
    stop();
         move_Stage=STAGE2;
    break;
    case STAGE2:
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING;setSteering(steeringAngle); /*turne right*/}
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING;setSteering(steeringAngle);/*turne left*/}

       if (encoder_count<50){forward(SHIFT_SPEED);}
       else{ move_Stage=STAGE3;}
       readDistance(LEFT_RIGHT);
       Serial.print("Range L= ");  Serial.print(rangeL);  Serial.print("   Range R= ");  Serial.print(rangeR);
       //encoderStright=constrain(rangeR, 15, 26);   encoderStrightOb
        if(rotionDirection==ANTI_CLOCKWIZE){encoderStrightOb=constrain(rangeL, 5, 20); encoderStrightOb=map(encoderStrightOb,5,20,750,100);}
        else if(rotionDirection==CLOCKWIZE){encoderStrightOb=constrain(rangeR, 5, 20);  encoderStrightOb=map(encoderStrightOb,5,20,800,300);}
         tempAng=currentAngle;
        Serial.print("   encoderStrightOb= ");  Serial.println(encoderStrightOb);
    break;
    case STAGE3:
    //steeringAngle!=STRAIGHT_STEERING-37){steeringAngle=STRAIGHT_STEERING-37; turnSteering(STRAIGHT_STEERING,steeringAngle,10); 
       if(rotionDirection==ANTI_CLOCKWIZE && steeringAngle!=STRAIGHT_STEERING+37){steeringAngle=STRAIGHT_STEERING+37; turnSteering(STRAIGHT_STEERING,steeringAngle,10); /*turne right*/}
       else if(rotionDirection==CLOCKWIZE && steeringAngle!=STRAIGHT_STEERING-37){steeringAngle=STRAIGHT_STEERING-37; turnSteering(STRAIGHT_STEERING,steeringAngle,10);/*turne left*/}

       if (encoder_count>200 && encoder_count<1100  && abs(currentAngle-tempAng)<85 ){forward(TURN_SPEED_L2);}
       else{ move_Stage=STAGE4; setSteering(STRAIGHT_STEERING); encoder_count=0; reverceTime=millis();}
    break;
    case STAGE4:
       
        if( readBackDistance(3)>=4 && (millis()-reverceTime)<1150 ){reverse(SPEED2);   digitalWrite(POS1,HIGH);
                                                                                       digitalWrite(POS2,HIGH);}
        else {stop(); move_Stage=STAGE6; shift_Stages = BRE_SHIFT; } 
       
        break;
    case STAGE5:  
                
        if(rotionDirection==ANTI_CLOCKWIZE){ shifRightCm(6);}
        else if(rotionDirection==CLOCKWIZE){ shifLeftCm(6);}
        
        if(shift_Stages ==  FINISH){stop(); move_Stage=STAGE6;}
    break;
    
    case STAGE6:

       Serial.println("Finish bre parking ");
         Parking_stages=CLEAR_SERIAL;
        obstacle_Stage=FINAL;
        motion_mode= PARKING;

    break;
    }
  }
void nff(){// update hhhhhheeeeaaarrr
  //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    currentPosition='F';
    switch(move_Stage){
    case STAGE1:
    stop();
         move_Stage=STAGE2;
        
    break;
    case STAGE2:
        tempAng=currentAngle;
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING;setSteering(steeringAngle); /*turne right*/}
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING;setSteering(steeringAngle);/*turne left*/}

       if (encoder_count<50){forward(SHIFT_SPEED);}
       else{stop(); delay(10); move_Stage=STAGE3;
              readDistance(LEFT_RIGHT);
       Serial.print("Range L= ");  Serial.print(rangeL);  Serial.print("   Range R= ");  Serial.print(rangeR);
       //encoderStright=constrain(rangeR, 15, 26);   encoderStrightOb


        if(rotionDirection==ANTI_CLOCKWIZE){encoderStrightOb=constrain(rangeL, 5, 20); encoderStrightOb=map(encoderStrightOb,5,20,600,50);}
        else if(rotionDirection==CLOCKWIZE){encoderStrightOb=constrain(rangeR, 5, 20);  encoderStrightOb=map(encoderStrightOb,5,20,750,200);}
        encoder_count;
       }

        
       Serial.print("   encoderStrightOb= ");  Serial.println(encoderStrightOb);
       
    break;
    case STAGE3:
    //steeringAngle!=STRAIGHT_STEERING-37){steeringAngle=STRAIGHT_STEERING-37; turnSteering(STRAIGHT_STEERING,steeringAngle,10); 
       if(rotionDirection==ANTI_CLOCKWIZE && steeringAngle!=STRAIGHT_STEERING+37){steeringAngle=STRAIGHT_STEERING+37; turnSteering(STRAIGHT_STEERING,steeringAngle,10); /*turne right*/}
       else if(rotionDirection==CLOCKWIZE && steeringAngle!=STRAIGHT_STEERING-37){steeringAngle=STRAIGHT_STEERING-37; turnSteering(STRAIGHT_STEERING,steeringAngle,10);/*turne left*/}
        readAngle(); 


        
        if (rotionDirection==CLOCKWIZE && encoder_count<1200  && abs(currentAngle-tempAng)<70 ){forward(TURN_SPEED_L2);  
         Serial.print("tempAng angle ="); Serial.println(tempAng);
        Serial.print("Dalta angle ="); Serial.print( abs(currentAngle-tempAng));    Serial.print("    Current angle ="); Serial.println(currentAngle); 
        }
       else if ( rotionDirection==ANTI_CLOCKWIZE &&  encoder_count<1200  && abs(currentAngle-tempAng)<65 ){forward(TURN_SPEED_L2);  
         Serial.print("tempAng angle ="); Serial.println(tempAng);
        Serial.print("Dalta angle ="); Serial.print( abs(currentAngle-tempAng));    Serial.print("    Current angle ="); Serial.println(currentAngle);
        }
       else{ move_Stage=STAGE4;   turnSteering(steeringAngle, STRAIGHT_STEERING,10);
                 Serial.print("First turne encoder_count ="); Serial.println(encoder_count);
                  Serial.print("tempAng ="); Serial.println(tempAng);
                    Serial.print("tempAng ="); Serial.println(abs(currentAngle-tempAng));
                 
         
       encoder_count=0;}
    break;
    case STAGE4:
           if (encoder_count<encoderStrightOb){forward(TURN_SPEED_L2);}
       else{ stop(); move_Stage=STAGE5; /*setSteering(STRAIGHT_STEERING);*/  encoder_count=0;   Serial.print("encoderStrightOb ="); Serial.println(encoderStrightOb); // motion_mode=STOP;
       }
    break;
    case STAGE5:  
       if(rotionDirection==ANTI_CLOCKWIZE && steeringAngle!=STRAIGHT_STEERING-37){steeringAngle=STRAIGHT_STEERING-37; turnSteering(STRAIGHT_STEERING,steeringAngle,10); /*turne left*/}
       else if(rotionDirection==CLOCKWIZE && steeringAngle!=STRAIGHT_STEERING+37){steeringAngle=STRAIGHT_STEERING+37; turnSteering(STRAIGHT_STEERING,steeringAngle,10);/*turne Right*/}
         
         readAngle(); 
       
       if (encoder_count<1000  && abs(currentAngle-tempAng)>4 ){forward(SLOW_SPEED2);}
       else{ stop(); move_Stage=STAGE6; setSteering(STRAIGHT_STEERING);  encoder_count=0;motion_mode=STOP;}


         if(rotionDirection==ANTI_CLOCKWIZE && abs((int)rangeR-15)>4 ){  siftDist=constrain(abs((int)rangeR-15), 2,15); 
                                                                         if((int)rangeR>15){shift="right";} else{shift="left";}  shift_Stages=BRE_SHIFT; }
      
         else if(rotionDirection==CLOCKWIZE && abs((int)rangeL-15)>4 ){  siftDist=constrain(abs((int)rangeL-15), 2,15); 
                                                                         if((int)rangeL>15){shift="left";} else{shift="right";}  shift_Stages=BRE_SHIFT; }
       
         else{shift="NO";}
          Serial.print("Range L ="); Serial.println(rangeL);
          Serial.print("Range R ="); Serial.println(rangeR);
          Serial.print("shift  = "); Serial.println(shift);
         // motion_mode=STOP;
         encoder_count=0;

       
    break;
    
    case STAGE6:

    /*
         if (encoder_count<300 ){currentPosition='F'; complementaryMode=false;setpoint=16; forward(SLOW_SPEED3);  setSteering(steeringPID);}
         else{move_Stage=STAGE7; setSteering(STRAIGHT_STEERING);  steeringAngle=STRAIGHT_STEERING;}*/
         currentPosition='F';
         
         if(shift=="left"){      shifLeftCm(siftDist);if(shift_Stages==FINISH){move_Stage=STAGE7;} }  // 
         else if(shift=="right"){  shifRightCm(siftDist);  if(shift_Stages==FINISH){move_Stage=STAGE7;}}
         else{
         if (encoder_count<300 ){  complementaryMode=false; setpoint=15; forward(SLOW_SPEED3);  setSteering(steeringPID);
         Serial.print("rangeR="); Serial.println(rangeR);
          Serial.print("setpoint="); Serial.println(setpoint);
          Serial.print("complementary mode="); Serial.println(complementaryMode);
          Serial.print("steeringPID="); Serial.println(steeringPID);
         }
         else{move_Stage=STAGE7;delay(100);  steeringAngle=STRAIGHT_STEERING;/*setSteering(STRAIGHT_STEERING);*/}
         }
    break;
    case STAGE7:
        currentPosition='F';
        Serial.println("Finish NFF");
        
       // motion_mode=FORWARD; move_Stage=STAGE1; 
         move_Stage=STAGE1; obstacle_Stage=FINAL;
        // motion_mode=STOP;
       // obstacle_Stage=FINAL; 
    break;
    }
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
  }


  //****
   //********************************************************************************************************
void fnn(){
   switch(move_Stage){
    case STAGE1:
         tempAng=currentAngle;
         currentPosition='F';
         move_Stage=STAGE2;
    break;
    case STAGE2:
       if(rotionDirection==ANTI_CLOCKWIZE){steeringAngle=STRAIGHT_STEERING; setSteering(steeringAngle); /*turne right*/}
       else if(rotionDirection==CLOCKWIZE){steeringAngle=STRAIGHT_STEERING; setSteering(steeringAngle);/*turne left*/}

       if (encoder_count<50){forward(SHIFT_SPEED);}
       else{ move_Stage=STAGE3;}

       Serial.print("Range L= ");  Serial.print(rangeL);  Serial.print("   Range R= ");  Serial.print(rangeR);
       //encoderStright=constrain(rangeR, 15, 26);   encoderStrightOb
        if(rotionDirection==ANTI_CLOCKWIZE){encoderStrightOb=constrain(rangeR, 5, 20); encoderStrightOb=map(encoderStrightOb,5,20,790,0);}
        else if(rotionDirection==CLOCKWIZE){encoderStrightOb=constrain(rangeL, 5, 20);  encoderStrightOb=map(encoderStrightOb,5,20,220,0);}
      // encoderStrightOb=0;
       Serial.print("   encoderStrightOb= ");  Serial.println(encoderStrightOb);
    break;
    case STAGE3:
       if(rotionDirection==ANTI_CLOCKWIZE && steeringAngle!=STRAIGHT_STEERING-37){steeringAngle=STRAIGHT_STEERING-37; turnSteering(STRAIGHT_STEERING,steeringAngle,10); /*turne right*/}
      else if(rotionDirection==CLOCKWIZE && steeringAngle!=STRAIGHT_STEERING+37){steeringAngle=STRAIGHT_STEERING +37;turnSteering(STRAIGHT_STEERING,steeringAngle,10);/*turne left*/}
      
        readAngle(); 
     
       if (encoder_count>200 && encoder_count<1100 && abs(currentAngle-tempAng)<95 ){forward(TURN_SPEED_L2);}
       else{ move_Stage=STAGE4; /*setSteering(STRAIGHT_STEERING);*/ turnSteering(steeringAngle,STRAIGHT_STEERING,10);encoder_count=0;  }
       Serial.print("   encoder_count= ");  Serial.println(encoder_count);
        Serial.print("   currentAngle= ");  Serial.println(currentAngle);
         Serial.print("  tempAng = ");  Serial.println(tempAng);

        // if(rounCounter==5){ESP.restart();}
    break;
    case STAGE4:
       //ESP.restart();
       if (encoder_count<encoderStrightOb ){forward(TURN_SPEED_L2);}
       else{ stop(); delay(50); move_Stage=STAGE5; setSteering(STRAIGHT_STEERING); encoder_count=0; /*motion_mode=STOP;*/}
    break;
    case STAGE5:
          currentPosition='N';
          setpoint=71;
       if(rotionDirection==ANTI_CLOCKWIZE && steeringAngle!=STRAIGHT_STEERING+37){steeringAngle=STRAIGHT_STEERING+37; turnSteering(STRAIGHT_STEERING,steeringAngle,10); /*turne left*/}
       else if(rotionDirection==CLOCKWIZE && steeringAngle!=STRAIGHT_STEERING-37){steeringAngle=STRAIGHT_STEERING-37; turnSteering(STRAIGHT_STEERING,steeringAngle,10);/*turne Right*/}
       readAngle();
       if (encoder_count<1000 && abs(currentAngle-tempAng)>4){forward(TURN_SPEED_L2);}
       else{ stop(); move_Stage=STAGE6; setSteering(STRAIGHT_STEERING);  // motion_mode=STOP;
        Serial.print("nxtAng ="); Serial.println(nxtAng);
                              //RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR
         if(rotionDirection==ANTI_CLOCKWIZE && abs((int)rangeL-15)>4 ){  siftDist=constrain(abs((int)rangeL-15), 2,15); 
                                                                         if((int)rangeL>15){shift="left";} else{shift="right";}  shift_Stages=BRE_SHIFT; }
      
         else if(rotionDirection==CLOCKWIZE && abs((int)rangeR-15)>4 ){  siftDist=constrain(abs((int)rangeR-15), 2,15); 
                                                                         if((int)rangeR>15){shift="right";} else{shift="left";}  shift_Stages=BRE_SHIFT; }
       
         else{shift="NO";}
          Serial.print("Range L ="); Serial.println(rangeL);
          Serial.print("Range R ="); Serial.println(rangeR);
          Serial.print("shift  = "); Serial.println(shift);
         // motion_mode=STOP;
         encoder_count=0;
       // ESP.restart();
       }
    break;
    case STAGE6:

       
        currentPosition='N';

         if(shift=="left"){      shifLeftCm(siftDist);if(shift_Stages==FINISH){move_Stage=STAGE7;} }  // 
         else if(shift=="right"){  shifRightCm(siftDist);  if(shift_Stages==FINISH){move_Stage=STAGE7;}}
         else{
         if (encoder_count<300 ){ currentPosition='N'; complementaryMode=true;setpoint=71; forward(SLOW_SPEED3);  setSteering(steeringPID);
         Serial.print("rangeR="); Serial.println(rangeR);
          Serial.print("setpoint="); Serial.println(setpoint);
          Serial.print("complementary mode="); Serial.println(complementaryMode);
          Serial.print("steeringPID="); Serial.println(steeringPID);
         }
         else{move_Stage=STAGE7;delay(100);  steeringAngle=STRAIGHT_STEERING;/*setSteering(STRAIGHT_STEERING);*/}
         }
         
         //preCornerPos='F';
    break;
    case STAGE7:
        currentPosition='N';
        Serial.println("Finesh FNN");
         move_Stage=STAGE1; obstacle_Stage=FINAL;
      //  motion_mode=FORWARD; move_Stage=STAGE1;   //motion_mode=STOP; 
    break;
   }
  }


void setObstaclePos(){
  if(obstacles[rounCounter%4].charAt(0)=='F'){currentPosition='F'; complementaryMode=false;setpoint=16;}
  else{currentPosition='N'; complementaryMode=true; setpoint=71;}
  obIndex=rounCounter%4;
  if(obstacles[obIndex]=="FFF"){obstacle_Stage=FFF;  Serial.println("FFF");} 
  else if(obstacles[obIndex]=="NFF"){obstacle_Stage=NFF; move_Stage = STAGE1;  Serial.println("NFF");}// 
  else if(obstacles[obIndex]=="NNF"){obstacle_Stage=NNF; move_Stage = STAGE1;Serial.println("NNF"); }//
  else if(obstacles[obIndex]=="NNN"){obstacle_Stage=NNN;Serial.println("NNN"); move_Stage = STAGE1;}
  else if(obstacles[obIndex]=="FNN"){obstacle_Stage=FNN; move_Stage = STAGE1; Serial.println("FNN");}//
  else if(obstacles[obIndex]=="FFN"){obstacle_Stage=FFN; move_Stage = STAGE1; Serial.println("FFN");}//  
  else if(obstacles[obIndex]=="NNNH"){obstacle_Stage=NNNH; move_Stage = STAGE1; Serial.println("NNNH bre parking");}//
  else if(obstacles[obIndex]=="NNNT"){obstacle_Stage=NNNT; move_Stage = STAGE1; Serial.println("NNNT bre parking");}//
  
  Serial.print("obstacles pos is"); Serial.println(obstacles[obIndex]);
  Serial.print("currentPosition : ");    Serial.println(currentPosition);
  encoder_count=0;
   // if(rounCounter==8){motion_mode=STOP;}
  }

void betweenOBbstacles(){



  
  switch(obstacle_Stage){  
    
    case FIRST:
             if(currentPosition=='F'){complementaryMode=false;setpoint=16;}
             else if(currentPosition=='N') { complementaryMode=true; setpoint=73;}
             setObstaclePos();
    break;
    case FFF:
       fff();
    break;
    case NFF:
       nff();
    break;
  
    case NNN:
       Serial.print("Complementary is : "); Serial.println(complementaryMode);
       Serial.print("setpoint is : "); Serial.println(setpoint);
      nnn();
    break;
    case NNNH:
       nnnH();
    break;//"NNNH"  "NNNT"
    case NNNT:
       nnnT();
    break;
    case FNN:
      fnn();
    break;
    case FINAL:
    //complementaryMode=false;
        motion_mode=FORWARD;
    break;
    }
 
  }
