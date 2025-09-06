#define R 0
#define L 1
#define TIME_PRELEARN 1000
#define LEARN_TIME  3000
#define TIME_AFTERLEARN 500
#define LERARN_TIMEOUT 5000
bool printAngles;
unsigned long startLearn,parkingTime;  
int shiftCM;    
unsigned long runTime2=0,delayTime;     
char currentPosition,nextPosition;
int C,U;
int Fact;
int strightEncoder1,strightEncoder2,nxtAng,tempAng;  //encoder values and next angle  for  level2 turn
int stright1PID,stright2PID;  //set point to go stright in corner regon
int readDistanseFrom;
unsigned long encoderCounterPreTurn=0;
bool testdirectionMore,near,printArray,UTurnState,afterUTurn;
int encoderLeft,encoderRight,encoderTurn1,encoderTurn2,encoderStright,encoderReviece,encoderStrightOb,encoderTurn;
int roundNo,turnNo;
int rDirection;
String Ustate[4];
String recivedMsg; // path of robot between ob.s 
String  obstacles[4];
String parking[4];
int fullRounds=12;
String  InverseObstacles[4];
int obIndex;

bool isStarted2, testDir=true;
int dist,preTurnDist;
double firstroundDest=0.0;

#include"betweenOb.h"

String inverseObs(String s){
  Serial.print("Original OB.s  is: " ); Serial.println(s);
  
  String rStr;
  String c1,c2,c3;
  c1=String(s.charAt(2));if(c1=="F"){c1="N"; }else {c1="F";}
  c2=String(s.charAt(1));if(c2=="F"){c2="N"; }else {c2="F";}
  c3=String(s.charAt(0)); if(c3=="F"){c3="N"; }else {c3="F";}
  rStr=c1+c2+c3;
  Serial.print("Inverted  is: " ); Serial.println(rStr);

  if(rStr=="NNF"){rStr="NFF";}
  else if(rStr=="FFN"){rStr="FNN";}
  ///must Correct 
  return rStr;
  }

void  FillObAndUturnState (String msg,int i){
  
  if(msg=="EEE"){Ustate[i]="M";}
  else if(msg.charAt(2)=='R'){Ustate[i]="Y";}
  else if(msg.charAt(2)=='G'){Ustate[i]="N";}
  else{Ustate[i]="N";}

  if(msg=="GGR"){     if(rotionDirection==ANTI_CLOCKWIZE){obstacles[i]="NFF"; }else{obstacles[i]="FNN" ;}   }
  else if(msg=="GRG"){ if(rotionDirection==ANTI_CLOCKWIZE){ obstacles[i]="NFF";}else{obstacles[i]="FNN" ; } }
  else if(msg=="RGG"){ if(rotionDirection==ANTI_CLOCKWIZE){ obstacles[i]="FNN";}else{obstacles[i]="NFF" ; } }
  else if(msg=="RRG"){ if(rotionDirection==ANTI_CLOCKWIZE){ obstacles[i]="FNN"; }else{obstacles[i]="NFF"; }}
  else if(msg=="RGR"){ if(rotionDirection==ANTI_CLOCKWIZE){ obstacles[i]="FNN";}else{obstacles[i]="NFF" ; } }
  else if(msg=="GRR"){ if(rotionDirection==ANTI_CLOCKWIZE){ obstacles[i]="NFF";}else{obstacles[i]="FNN" ;}   }
  else if(msg=="RRR"){ if(rotionDirection==ANTI_CLOCKWIZE){ obstacles[i]="FFF"; }else{obstacles[i]="NNN" ; } }
  else if(msg=="GGG"){ if(rotionDirection==ANTI_CLOCKWIZE){ obstacles[i]="NNN";}else{obstacles[i]="FFF" ; } }
  else if(msg=="EEE"){ if(rotionDirection==ANTI_CLOCKWIZE){ obstacles[i]="FFF";}else{obstacles[i]="FFF" ; } }
  else{                                                     obstacles[i]="FFF"; }
   InverseObstacles[i]=inverseObs(obstacles[i]);
    Serial.println("_____________________________________________");
    Serial.print("Obestical path ");  Serial.println(obstacles[i]);
    Serial.print("Inverse Obestical path ") ; Serial.println(InverseObstacles[i]);
    Serial.println("_____________________________________________");


}


bool checkUturn(){
  bool uState=false;
  if(Ustate[0]=="Y"){uState=true;}
  else if(Ustate[0]=="N"){uState=false;}
  else{ 
     if(Ustate[3]=="Y"){uState=true;}
     else if(Ustate[3]=="N"){uState=false;}
     else{
         if(Ustate[2]=="Y"){uState=true;}
         else if(Ustate[2]=="N"){uState=false;}
         else{
             if(Ustate[1]=="Y"){uState=true;}
             else if(Ustate[1]=="N"){uState=false;}
             else {uState=false;}
         }
      }
    }
   return uState;
}
void debugLevel2(){
  
  Serial.print("encoder_count: "); Serial.print( encoder_count);
  Serial.print(": "); Serial.print( rounCounter);
  Serial.print("setpoint: "); Serial.print( setpoint);
  Serial.println();
  }
void initLevel2(){
  testDir=true;
   printAngles=true;
   parking[0]=parking[1]=parking[2]=parking[3]="";
   parking[1]="H";
  ////////////////////
        //  obstacles[1]="NNN";   InverseObstacles[1]= "FFF";
        //  obstacles[2]="FFF";   InverseObstacles[2]="NNN";
        //  obstacles[3]="NNN";    InverseObstacles[3]= "FFF";
        //  obstacles[0]="FFF";     InverseObstacles[0]="NNN";

         // Serial.print("ob1:  ");  Serial.print(obstacles[1]);  Serial.print("inverse ob1:  "); Serial.println(InverseObstacles[1]);
  ////////////////////
  UTurnState=true;
  afterUTurn=false;
  obIndex=0;
   C=U=0;
   readDistanseFrom=LEFT_RIGHT;
   //readDistanseFrom=LIFT_FORWARD ;
   //readDistanseFrom=RIGHT_FORWARD ;
   rDirection=0;
   near=false;
   testdirectionMore=false;
   isStarted2=false;
  // currentAngle=(int)mpu.GetAngZ();
//   currentAngle=(int)mpu.getAngleZ();
  readAngle();
 //  debugRotation();
   initAngle=currentAngle;
   angle0=currentAngle;
   motion_mode=STANDBY;
   initR= rangeR;
   initL= rangeL;
   initF= rangeF;
   roundNo=turnNo=0;
   rounCounter=0;
   rotionDirection=UNKNOWN_DIRECTION;
   setpoint = initR;
   dist=40;

   //***********
   /*
     move_Stage=STAGE1; 
    rounCounter=8;
    UTurnState=true;
    currentPosition='N';
    //rotionDirection=CLOCKWIZE;
    rotionDirection=ANTI_CLOCKWIZE;
    */
    // readDistanseFrom=LEFT_RIGHT;
   ///////////////*****************************************************
  }
///For level 2 U Turn  For level 2 U Turn   For level 2 U Turn  For level 2 U Turn  For level 2 U Turn  

int invertDirection(int currentDirection){
  if (setpoint==71){setpoint=15;}
  else if(setpoint==15){setpoint=71;}
  return(!currentDirection);
  }

void inverceParking(){
  for (int i =0 ;i<4;i++){
    if(i==0 ||  i==2){if(parking[i]=="T"){parking[i]="H"; break; }
                      else if(parking[i]=="H"){parking[i]="T"; break; }
                      }
    else if(i==1 ){if(parking[i]=="T"){parking[1]=""; parking[3]="H"; break;}
              else if(parking[i]=="H"){parking[1]="";  parking[3]="T";  } break;}
    else if(i==3 ){if(parking[i]=="T"){parking[3]=""; parking[1]="H"; break;}
              else if(parking[i]=="H"){parking[3]="";  parking[1]="T";  } break;}
 
    }
  
  }

void UTurnCalculations(char currentPos){
  char nextPos;
  preUTurneAngle=currentAngle;
  if(currentPos=='N'){nextPos='F';} else{nextPos='N';}
  Serial.println("Convert pos ....");
  //int nextDirection=rotionDirection;

      Serial.print ("Direction is "); Serial.println(rotionDirection);
       Serial.print ("currentPos is "); Serial.println(currentPos);
      if(rotionDirection==ANTI_CLOCKWIZE){
        if(currentPos=='N'){setpoint=71;  UTurnDirection=UTURN_RIGHT; nextAngle=preUTurneAngle-180; rotitionEncoder=3000; Serial.println("Uturne Right");}//U turne right
        else if(currentPos=='F'){setpoint=15; UTurnDirection=UTURN_LEFT; nextAngle=preUTurneAngle+180; rotitionEncoder=3100; Serial.println("Uturne Left");} //U turne left
        }
      else if(rotionDirection==CLOCKWIZE){
        if(currentPos=='N'){setpoint=71; UTurnDirection=UTURN_LEFT; nextAngle=preUTurneAngle+180;rotitionEncoder=3100;  Serial.println("Uturne Left");}//U turne left
        else if(currentPos=='F'){setpoint=15; UTurnDirection=UTURN_RIGHT; nextAngle=preUTurneAngle-180;rotitionEncoder=3000;Serial.println("Uturne Right");} //U turne righ
        }
    
  }

void doUTurn(){
 

   switch(uTurne_Stages){
    case CALCULATIONS:
        UTurnCalculations(currentPosition);  uTurne_Stages=STRAIGHT2; setSteering(STRAIGHT_STEERING); encoder_count=0;
        Serial.println("End calculations");
        
    break;
    case STRAIGHT1:
         Serial.println("Stright 1");
       if (encoder_count<300){forward(SPEED2);}
       else{ uTurne_Stages=STRAIGHT2; encoder_count=0;}          
    break;
    case STRAIGHT2: 
     Serial.println("Stright 2");
       if (encoder_count<1000){forward(SPEED2);setSteering(steeringPID);}
       else{ stop(); uTurne_Stages=ROTATE; encoder_count=0;} 
                    
    break;
    case ROTATE:
     Serial.println("Rotate");
        if(UTurnDirection==UTURN_RIGHT && steeringAngle!=STRAIGHT_STEERING+20){steeringAngle=STRAIGHT_STEERING+20; turnSteering(steeringPID,steeringAngle,10);}//kkkkkkkkkkkkkkkkkkkRRRRRRRRRRRRRRRRRRRRRR
        else if  (UTurnDirection==UTURN_LEFT && steeringAngle!=STRAIGHT_STEERING-20){steeringAngle=STRAIGHT_STEERING-20;turnSteering(steeringPID,steeringAngle,10);}
         readAngle();
        
        if (encoder_count<2800 && abs(currentAngle-nextAngle)>5){if(encoder_count<1900){forward(SPEED2);} else {forward(SLOW_SPEED3);}}
        else{ stop(); delay(50); uTurne_Stages=STRAIGHT3; turnSteering(steeringAngle,STRAIGHT_STEERING,10);
       if(currentPosition=='N'){currentPosition='F';}else {currentPosition='N';} complementaryMode=false;
        rotionDirection=invertDirection(rotionDirection);
       
        Serial.print("nextAngle");Serial.println(nextAngle);
        Serial.print("currentAngle");Serial.println(currentAngle);
        Serial.print("encoder_count");Serial.println(encoder_count);   
         Serial.print("preUTurneAngle");Serial.println(preUTurneAngle);
         encoder_count=0;
        // ESP.restart();
        } 
    break;
    case STRAIGHT3:
       if(currentPosition=='F'){complementaryMode=false; setpoint=16;}
       else if(currentPosition=='N') { complementaryMode=false; setpoint=73;}
                 
       if (encoder_count<500){forward(SLOW_SPEED2);setSteering(steeringPID);}
       else{ stop(); uTurne_Stages=FINISH_UTURN; encoder_count=0;afterUTurn=true; nxtAng =nextAngle;} 
  
    break;
    case FINISH_UTURN:
   // Serial.println("Move Direction ");
    inverceParking();
    Serial.print("preUTurneAngle") ; Serial.println(preUTurneAngle); 
    Serial.print("nextAngle") ; Serial.println(nextAngle);
    Serial.print("currentAngle") ; Serial.println(currentAngle);
    Serial.print("currentPos") ; Serial.println(currentPosition);
    Serial.print("rotionDirection") ; Serial.println(rotionDirection); 
    
     move_Stage=STAGE4;
      encoder_count=0;
   // ESP.restart();
    break;
   }
   
  }
/// End level 2 U Turn 

/*
shift_Stages = TURN1
 STRIGHT,
  FINISH,
  steeringAngle=STRAIGHT_STEERING-43;  setSteering(steeringAngle);} //turn Steering  Left

*/
void GoComplementary(){
  
   switch(shift_Stages){
    case BRE_SHIFT:
    
       encoder_count=0;
       complementaryMode=false;
       rotionDirection=CLOCKWIZE;
       setSteering(STRAIGHT_STEERING);
       setpoint=73;
       shift_Stages=TURN1;
    break;
    case TURN1:
     debugUltrasonic();
    if(encoder_count>2000){shift_Stages=STRIGHT;stop();}
    forward(SPEED2);
    setSteering(steeringPID);
    break;
    case STRIGHT:
     motion_mode=STOP;
    break;      
    }
   
   }
////


bool dalta(int side){
  int del;
    readDistanseFrom=LEFT_RIGHT;
   readDistance(readDistanseFrom);
    initR=rangeR;    initL=rangeR; 
  if (side==R){ del =initR-rangeR; if(abs(del)>=20)return true; else return false;}
  else{  del = initL-rangeL;  if(abs(del)>=20) return true; else return false;}
  }



// function to find Direction 
void detectPos(){
        if(rotionDirection==CLOCKWIZE){
        if(rangeL>=1 && rangeL<=26){currentPosition='F';  digitalWrite(POS1,LOW);  digitalWrite(POS2,HIGH);}
        else if(rangeL>=41 && rangeL<=48){currentPosition='C'; digitalWrite(POS1,HIGH);  digitalWrite(POS2,LOW);}
        else if(rangeL>=60 && rangeL<=79){currentPosition='N';   digitalWrite(POS1,LOW);  digitalWrite(POS2,LOW);}
        } 
      else if(rotionDirection==ANTI_CLOCKWIZE){
       Serial.print(" initR: ");Serial.println(initR);
        if(rangeR>=60 && rangeR<=79){currentPosition='N';  digitalWrite(POS1,LOW);  digitalWrite(POS2,LOW);}
        else if(rangeR>=40 && rangeR<60){currentPosition='C'; digitalWrite(POS1,HIGH);  digitalWrite(POS2,LOW);}
        else if(rangeR<30){currentPosition='F'; digitalWrite(POS1,LOW);  digitalWrite(POS2,HIGH);}
        
        }
        Serial.print("rangeL");  Serial.println(rangeL);
        Serial.print("rangeR");  Serial.println(rangeR);
        Serial.print("Position is ");  Serial.println(currentPosition);

  }

void detectDirection(){//debugUltrasonic();

     switch(detect_direction){
    case FIRST_READ:
    forward(SLOW_SPEED);
    
    Serial.println("FIRST_READ");
     setSteering(STRAIGHT_STEERING);
     readDistanseFrom=ALL;
      readDistance(readDistanseFrom);
        Serial.print("rangeL");  Serial.println(rangeL);
        Serial.print("rangeR");  Serial.println(rangeR);
        Serial.print("rangeF");  Serial.println(rangeF);
   if(rangeR>110  || rangeF<95 ){C++; led(WHITE);  Serial.println("Direction may be CLOCK wize");  stop();  encoder_count=0; detect_direction=GO_LITTLE;  delayTime=millis(); }
   else if(rangeL >110 || rangeF<95){U++; rDirection=rDirection+1;  led(WHITE);  Serial.println("Direction UntiClock may be"); encoder_count=0; detect_direction=GO_LITTLE;  delayTime=millis();}
    

    break;
        case GO_LITTLE:
      
       // Serial.println("GO_LITTLE");
      readDistance(readDistanseFrom);
      if(encoder_count<250 ){forward(SLOW_SPEED); }
      else{detect_direction=DETECT_POSITION; stop(); 
        Serial.print("rangeL");  Serial.println(rangeL);
        Serial.print("rangeR");  Serial.println(rangeR);
      }

    break;

        case DETECT_POSITION:
    
         U=C=0;
         for (int i=0;i<5;i++){
         readDistanseFrom=LEFT_RIGHT; delay(5);
         readDistance(readDistanseFrom);     
         if(rangeL >110  ){ U++;  led(WHITE); }
         if(rangeR>110 ){ C++;   led(WHITE); }
         
         Serial.print("rangeL");  Serial.println(rangeL);
        Serial.print("rangeR");  Serial.println(rangeR);
        Serial.print("U   ");  Serial.println(U);
        Serial.print("C   ");  Serial.println(C);
        }

        if(U>=4){rotionDirection=ANTI_CLOCKWIZE; move_Stage=STAGE1;Serial.println("Direction UntiClock");  detectPos(); detect_direction=FINISH_DITECT; }
        else if (C>=4){rotionDirection=CLOCKWIZE;move_Stage=STAGE1; Serial.println("Direction CLOCK wize");  detectPos();detect_direction=FINISH_DITECT;  }
        else{Serial.print("UNKNOWN DIRECTION");detect_direction=REVERSE_LITTELE; encoder_count=0; }
        
        
    break;
        case REVERSE_LITTELE:
        if(encoder_count<270){reverse(SLOW_SPEED);}
        else{detect_direction=FIRST_READ;}

    break;
       break;
        case FINISH_DITECT:
            stop();

    break;
     }


/*
  

   
   if(millis()-delayTime>100){ forward(SLOW_SPEED); }
   else{testdirectionMore=true; stop();}
    
    if( testdirectionMore){stop();  testdirectionMore=false; 
      
      for (int i=0;i<5;i++){
         readDistanseFrom=LEFT_RIGHT; delay(20);
     //    readDistance(readDistanseFrom);    initR=rangeR;    initL=rangeR;  led(WHITE);
         if(rangeL >110  ){ U++;  rDirection=rDirection+0; led(WHITE); }
         if(rangeR>110 ){ C++;  rDirection=rDirection+1;  led(WHITE); }
                 Serial.print("rangeL");  Serial.println(rangeL);
        Serial.print("rangeR");  Serial.println(rangeR);
          Serial.print("U   ");  Serial.println(U);
          Serial.print("C   ");  Serial.println(C);
        }
    
        if(U==5){rotionDirection=ANTI_CLOCKWIZE; move_Stage=STAGE1;Serial.println("Direction UntiClock"); forward(SPEED2);}
        else if (C==5){rotionDirection=CLOCKWIZE;move_Stage=STAGE1; Serial.println("Direction CLOCK wize"); forward(SPEED2);}
        else {rotionDirection=UNKNOWN_DIRECTION;}

        
        if(rotionDirection==ANTI_CLOCKWIZE){ Serial.println("ANTI CLOCK WIZE");if(initL<25)near=true; readDistanseFrom=RIGHT_FORWARD;  motion_mode=STOP;}
        else if(rotionDirection==CLOCKWIZE){ Serial.println("CLOCK WIZE");if(initR<25)near=true; setpoint=initL; readDistanseFrom=LIFT_FORWARD ;motion_mode=STOP;}
        /*
         else if(rotionDirection==CLOCKWIZE){
        if(rangeL>=1 && rangeL<=26){currentPosition='F';  digitalWrite(POS1,LOW);  digitalWrite(POS2,HIGH);}
        else if(rangeL>=41 && rangeL<=48){currentPosition='C'; digitalWrite(POS1,HIGH);  digitalWrite(POS2,LOW);}
        else if(rangeL>=60 && rangeL<=79){currentPosition='N';   digitalWrite(POS1,LOW);  digitalWrite(POS2,LOW);}
        } 
      else if(rotionDirection==ANTI_CLOCKWIZE){
       Serial.print(" initR: ");Serial.println(initR);
        if(rangeR>=60 && rangeR<=79){currentPosition='N';  digitalWrite(POS1,LOW);  digitalWrite(POS2,LOW);}
        else if(rangeR>=40 && rangeR<60){currentPosition='C'; digitalWrite(POS1,HIGH);  digitalWrite(POS2,LOW);}
        else if(rangeR<30){currentPosition='F'; digitalWrite(POS1,LOW);  digitalWrite(POS2,HIGH);}
        
        }
        Serial.print("rangeL");  Serial.println(rangeL);
        Serial.print("rangeR");  Serial.println(rangeR);
        Serial.print("Position is ");  Serial.println(currentPosition);

        
        
       
    }*/
  }



// function to Enter Corner and stop
void doEnterCorner(){
   
  readDistanseFrom=ALL;
  readDistance(readDistanseFrom);
  //forward(SLOW_SPEED);SPEED2
  forward(SPEED2);
  Serial.print("rangeF=");   Serial.print(rangeF);
  encoder_count=0;
  if((rotionDirection==ANTI_CLOCKWIZE && rangeL>120) && rangeF<95){stop();move_Stage=STAGE2;}
  else if((rotionDirection==CLOCKWIZE && rangeR>120) &&  rangeF<95){stop();move_Stage=STAGE2;}
                 
  
  } 



   
    
void doForward2(){
  
  
  
  if(rotionDirection==UNKNOWN_DIRECTION){  setSteering(STRAIGHT_STEERING);  detectDirection(); } // if direction not set
  else{
    if(rounCounter<4 ){

    
    switch(move_Stage){
    case STAGE1:
             readDistanseFrom=ALL;
             readDistance(readDistanseFrom);
             
             if(rounCounter==0){setSteering(STRAIGHT_STEERING);}
             else{
                 if(currentPosition=='F'){complementaryMode=false; setpoint=16;}
                 else if(currentPosition=='N') { complementaryMode=true; setpoint=73;}//??????????????????????????
                 setSteering(steeringPID);
             }

          forward(SLOW_SPEED4);
         // Serial.print("rangeF=");   Serial.print(rangeF);
          //Serial.print("rangeF=");   Serial.print(rangeF);
          // Serial.print("rangeF=");   Serial.print(rangeF);
          encoder_count=0;
          if(rounCounter==2){  Serial.println("Forward untel  enter corner  in 33333333333333333333333");}
          if((rotionDirection==ANTI_CLOCKWIZE && rangeL>120) && rangeF<95){stop(); delay(50); complementaryMode=false; move_Stage=STAGE2;
                                                                      Serial.print("Forward distance is : "); Serial.println(readForwardDistance(5));
                                                                      Serial.println("Direction detected Anti");}
          else if((rotionDirection==CLOCKWIZE && rangeR>120) &&  rangeF<95) {stop(); delay(50);  complementaryMode=false;move_Stage=STAGE2;
                                                                      Serial.print("Forward distance is : ");  Serial.println(readForwardDistance(5));
                                                                      Serial.println("Direction detected Clock");}
         ///************
         encoder_count=0;
         
    break;
    case STAGE2:
     if(rounCounter==0){encoder_count=100;}
     if(rounCounter==2){  Serial.println("Forward 100 step  in 33333333333333333333333");}
        forward(SLOW_SPEED3);
        CurrentSteeringAng=STRAIGHT_STEERING;
       
        if(encoder_count>=100 ){stop(); move_Stage=STAGE3;delayTime=millis(); Serial.println("Forward 100 step"); }
      
    
    break;
    case STAGE3:
          if(rounCounter==2){ /*motion_mode=STOP;*/ Serial.println("Ready to learn in 33333333333333333333333");}
          if(millis()-delayTime>=TIME_PRELEARN){move_Stage=STAGE4;Serial.println("Look to turn learn");}
          
         if(rotionDirection==ANTI_CLOCKWIZE && steeringAngle!=STRAIGHT_STEERING-37){steeringAngle=STRAIGHT_STEERING-37; turnSteering(STRAIGHT_STEERING,steeringAngle,10); /*setSteering(steeringAngle);*/} //turn CAM LEFT    setSteering(STRAIGHT_STEERING);
         else if(rotionDirection==CLOCKWIZE&& steeringAngle!=STRAIGHT_STEERING+37){steeringAngle=STRAIGHT_STEERING+37; turnSteering(STRAIGHT_STEERING,steeringAngle,10); /* setSteering(steeringAngle);*/} // turn CAM RIGHT
    break;
    case STAGE4:
         motion_mode=LEARN;                      
         move_Stage=STAGE1;
      // motion_mode=STOP;
    break;
 
    }
    }
    //KKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKK   rounCounter==9
          //************************************************************************************************************************
     else if(rounCounter==8 && UTurnState){
      Serial.println("we  are in Final==>>   round  WITH u turne");
    
         
       
         switch(move_Stage){
          case STAGE1:
             Serial.println("Stage 1");
             if(currentPosition=='F'){complementaryMode=false; setpoint=14;}
             else if(currentPosition=='N') { complementaryMode=true; setpoint=71;}
              readDistanseFrom=ALL;
             setSteering(steeringPID);
             //setSteering(STRAIGHT_STEERING);
         ///************
         
         
        //  readDistance(readDistanseFrom);
          forward(SLOW_SPEED2);
          //forward(SPEED2);
          Serial.print("rangeF=");   Serial.print(rangeF);
          encoder_count=0;
          Serial.println("Chchchchchchchchchchchchchchchchchchchchchchchchchhchchchchchchchchhchchcchchchchchchhcchchchch");
          if((rotionDirection==ANTI_CLOCKWIZE && rangeL>120) && rangeF<95){stop(); complementaryMode=false;  move_Stage=STAGE2;}
          else if((rotionDirection==CLOCKWIZE && rangeR>120) &&  rangeF<95){stop();complementaryMode=false;  move_Stage=STAGE2;}
         ///************
         encoder_count=0;
         
    break;
    case STAGE2:
        forward(SLOW_SPEED2);
        if(encoder_count>=100){stop(); move_Stage=STAGE3;delayTime=millis(); 
          Serial.println("Do U Turn HAHAHAHAHAHAHAHAHAAHAHAHAHAH......"); /*   motion_mode=STOP;*/
          uTurne_Stages=CALCULATIONS; move_Stage=STAGE3;  Serial.println("DO u turne  DO u turne  DO u turne  DO u turne  DO u turne  DO u turne ");
        
          }
    
    break;
    case STAGE3:/// Do u turn
                
               doUTurn(); 

    break;
    case STAGE4:// go between obestacles 
             Serial.println("Finish 8 uturn okokokokokokok go 100 step ");
             
            
                      
          if(currentPosition=='F'){complementaryMode=false; setpoint=16;}
          else if(currentPosition=='N') { complementaryMode=false; setpoint=73;}//??????????????????????????
          forward(SLOW_SPEED2);
          setSteering(steeringPID);
             
           if( encoder_count>100){ stop();   move_Stage=STAGE5;  encoder_count=0;}
         
             //************
          /*
           if(rangeB>85){
                    stop(); Serial.println("Finish U turn");
                      obstacles[0]=InverseObstacles[0];
                      obstacles[1]=InverseObstacles[3];
                      obstacles[2]=InverseObstacles[2];
                      obstacles[3]=InverseObstacles[1];
                      Serial.println("ob 1 "); Serial.println( obstacles[1]);
                       Serial.println("ob 2 "); Serial.println( obstacles[2]);
                        Serial.println("ob 3 "); Serial.println( obstacles[3]);
                         Serial.println("ob 0"); Serial.println( obstacles[0]);
                        // motion_mode=STOP;
                         move_Stage=STAGE5;  encoder_count=0;
                      

                    }*/
             //************
   
    break;

     case STAGE5:// Enter corner section 
          readDistanseFrom=ALL;
          obstacle_Stage=FIRST; 
          
          move_Stage=STAGE6;
          encoder_count=0;
          Serial.println("Btween obestacles");
                              stop(); Serial.println("Finish U turn");
                      obstacles[0]=InverseObstacles[0];
                      obstacles[1]=InverseObstacles[3];
                      obstacles[2]=InverseObstacles[2];
                      obstacles[3]=InverseObstacles[1];
                      Serial.println("ob 1 "); Serial.println( obstacles[1]);
                       Serial.println("ob 2 "); Serial.println( obstacles[2]);
                        Serial.println("ob 3 "); Serial.println( obstacles[3]);
                         Serial.println("ob 0"); Serial.println( obstacles[0]);
          
        //  motion_mode=STOP;
    break;

      case STAGE6:
          Serial.println("Between obs after u turne......................................................");
          UTurnState=false;
          obstacle_Stage=FIRST;
          motion_mode=BTWEEN_OB;///QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ
          Serial.print("Next move is ");  Serial.println(obstacles[rounCounter%4]);
           if(rotionDirection==ANTI_CLOCKWIZE ){Serial.println("Direction is AntiClock");}
           else{ Serial.println("Direction is CLOCK");}
           Serial.print("Current pos is :");Serial.println(currentPosition);
           Serial.print("nextAngle");Serial.println(nextAngle);
           Serial.print("currentAngle");Serial.println(currentAngle);
           
      //  motion_mode=STOP;
        
    break;
    /*  case STAGE7:
         Serial.println();
         motion_mode=TURNE;
         if(rounCounter==8) {UTurnState=false;} 
         move_Stage=STAGE1;
         
         motion_mode=STOP;
    break;*/
 
    }
    
   
    
      }
    //KKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKK
     
        else if(rounCounter>=4 && rounCounter<12  ){
      Serial.println("we  are in second round or more without u turne");
      

       //ppppppppppppppppppppppppppppppppppppp
         switch(move_Stage){
          case STAGE1:
          
             if(currentPosition=='F'){complementaryMode=false; setpoint=14;}
             else if(currentPosition=='N') { complementaryMode=true; setpoint=71;}
             setSteering(steeringPID);
             //setSteering(STRAIGHT_STEERING);
         ///************
         
          readDistanseFrom=ALL;
        //  readDistance(readDistanseFrom);
          forward(SLOW_SPEED4);
          //forward(SPEED2);
          Serial.print("rangeF=");   Serial.print(rangeF);
          encoder_count=0;
          if((rotionDirection==ANTI_CLOCKWIZE && rangeL>120) && rangeF<95){stop(); delay(50); complementaryMode=false;  move_Stage=STAGE2;}
          else if((rotionDirection==CLOCKWIZE && rangeR>120) &&  rangeF<95){stop(); delay(50); complementaryMode=false;  move_Stage=STAGE2;}
         ///************
         encoder_count=0;
         
    break;
    case STAGE2:
        forward(SLOW_SPEED3);
        if(encoder_count>=100){stop(); move_Stage=STAGE3;delayTime=millis();}
    
    break;
    case STAGE3:
    // turn stearing to correct side
         if(millis()-delayTime>=TIME_AFTERLEARN){move_Stage=STAGE4;}
          //***
          Serial.print("ROUND 2===>>>  "); Serial.print("memory location=");  Serial.println((rounCounter+1)%4); 
          Serial.print("memory Data ="); Serial.println(obstacles[(rounCounter+1)%4]);   
          
         if(obstacles[(rounCounter+1)%4].charAt(0)=='F'){turnSteering(steeringPID,STRAIGHT_STEERING,10);}
         else{
               if(rotionDirection==ANTI_CLOCKWIZE && steeringAngle!=STRAIGHT_STEERING-37){steeringAngle=STRAIGHT_STEERING-37; turnSteering(STRAIGHT_STEERING,steeringAngle,10);/* setSteering(steeringAngle);*/} //turn Steering  Left 
               else if(rotionDirection==CLOCKWIZE && steeringAngle!=STRAIGHT_STEERING+37 ){steeringAngle=STRAIGHT_STEERING+37; turnSteering(STRAIGHT_STEERING,steeringAngle,10);/* setSteering(steeringAngle);*/} // turn steering Right
          }
          //***

    break;
    case STAGE4:

         motion_mode=TURNE;
         move_Stage=STAGE1;
    //  Serial.println("Pre second turne...................");
      //motion_mode=STOP;
    break;
        case STAGE7:
         if(rounCounter==8) {UTurnState=false;} 
         motion_mode=TURNE;
         move_Stage=STAGE1;
    //  Serial.println("Pre second turne...................");
      //motion_mode=STOP;
    break;
  
 
    }
    //PPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
   
      // motion_mode=STOP;
      }

      /// hhhh

      //************************************************************************************************************************
    else{  
      if(rounCounter==12){
          obstacles[1]="NNN";  
          obstacles[2]="NNN";  
          obstacles[3]="NNN";  
          obstacles[0]="NNN"; 

         if( parking[1]=="" && parking[2]=="" && parking[3]=="" && parking[0]=="" ){
         Serial.println("Iam Finish all of rounds and parking"); Serial.print("rounCounter"); Serial.println(rounCounter);stop(); motion_mode=STOP;}

         
         //calculate fullRounds and final parking "NNNH"  "NNNT"
         fullRounds=12;

         if( parking[1]!=""){      fullRounds=13;  if (parking[1]=="H"){ obstacles[1]="NNNH";} else{ obstacles[1]="NNNT";}  }
         else if( parking[2]!=""){ fullRounds=14;  if (parking[2]=="H"){ obstacles[2]="NNNH";} else{ obstacles[2]="NNNT";} }
         else if( parking[3]!=""){ fullRounds=15;  if (parking[3]=="H"){ obstacles[3]="NNNH";} else{ obstacles[3]="NNNT";} } 
         else if( parking[0]!=""){ fullRounds=16;  if (parking[0]=="H"){ obstacles[0]="NNNH";} else{ obstacles[0]="NNNT";} }

         Serial.print("obeatacles[1] : ");  Serial.println(obstacles[1]);
         Serial.print("obeatacles[2] : ");  Serial.println(obstacles[2]);
          Serial.print("obeatacles[3] : ");  Serial.println(obstacles[3]);
           Serial.print("obeatacles[0] : ");  Serial.println(obstacles[0]);

         Serial.print("Full rounds to parking is : ");  Serial.println(fullRounds);
        
          
        }

   



          
         if(rounCounter==fullRounds && fullRounds!=12){motion_mode=PARKING;}
         //======================================================================================================================
         //For parking  For parking  For parking  For parking  For parking  For parking   For parking   For parking   For parking
         
         switch(move_Stage){
          case STAGE1:
          
             if(currentPosition=='F'){complementaryMode=false; setpoint=14;}
             else if(currentPosition=='N') { complementaryMode=true; setpoint=71;}
             setSteering(steeringPID);
             //setSteering(STRAIGHT_STEERING);
         ///************
         
          readDistanseFrom=ALL;
        //  readDistance(readDistanseFrom);
          forward(SLOW_SPEED4);
          //forward(SPEED2);
          Serial.print("rangeF=");   Serial.print(rangeF);
          encoder_count=0;
          if((rotionDirection==ANTI_CLOCKWIZE && rangeL>120) && rangeF<95){stop(); delay(50); complementaryMode=false;  move_Stage=STAGE2;}
          else if((rotionDirection==CLOCKWIZE && rangeR>120) &&  rangeF<95){stop(); delay(50); complementaryMode=false;  move_Stage=STAGE2;}
         ///************
         encoder_count=0;
         
    break;
    case STAGE2:
        forward(SLOW_SPEED3);
        if(encoder_count>=100){stop(); move_Stage=STAGE3;delayTime=millis();}
    
    break;
    case STAGE3:
    // turn stearing to correct side
         if(millis()-delayTime>=TIME_AFTERLEARN){move_Stage=STAGE4;}
          //***
          Serial.print("ROUND 2===>>>  "); Serial.print("memory location=");  Serial.println((rounCounter+1)%4); 
          Serial.print("memory Data ="); Serial.println(obstacles[(rounCounter+1)%4]);   
          
         if(obstacles[(rounCounter+1)%4].charAt(0)=='F'){turnSteering(steeringPID,STRAIGHT_STEERING,10);}
         else{
               if(rotionDirection==ANTI_CLOCKWIZE && steeringAngle!=STRAIGHT_STEERING-37){steeringAngle=STRAIGHT_STEERING-37; turnSteering(STRAIGHT_STEERING,steeringAngle,10);/* setSteering(steeringAngle);*/} //turn Steering  Left 
               else if(rotionDirection==CLOCKWIZE && steeringAngle!=STRAIGHT_STEERING+37 ){steeringAngle=STRAIGHT_STEERING+37; turnSteering(STRAIGHT_STEERING,steeringAngle,10);/* setSteering(steeringAngle);*/} // turn steering Right
          }
          //***

    break;
    case STAGE4:

         motion_mode=TURNE;
         move_Stage=STAGE1;

    break;

  
 
    }
         //For parking  For parking  For parking  For parking  For parking  For parking   For parking   For parking   For parking
           //======================================================================================================================
        
         
    
        }
     

    
    
  
  
  
}

}


void  OB_Learn(){
  
    switch(move_Stage){
    case STAGE1:
    //+++++++++++++++++++++++++++++++++++++++++++++++++++=
     readDistance(LEFT_RIGHT);
          if(rotionDirection==CLOCKWIZE){
        
        if(rangeL>=1 && rangeL<=26){currentPosition='F';  digitalWrite(POS1,LOW);  digitalWrite(POS2,HIGH);}
        else if(rangeL>=41 && rangeL<=48){currentPosition='C'; digitalWrite(POS1,HIGH);  digitalWrite(POS2,LOW);}
        else if(rangeL>=60 && rangeL<=79){currentPosition='N';   digitalWrite(POS1,LOW);  digitalWrite(POS2,LOW);}
        } 
      else if(rotionDirection==ANTI_CLOCKWIZE){
       Serial.print(" initR: ");Serial.println(initR);
        if(initR>=60){currentPosition='N';  digitalWrite(POS1,LOW);  digitalWrite(POS2,LOW);}
        else if(initR>=40 && initR<60){currentPosition='C'; digitalWrite(POS1,HIGH);  digitalWrite(POS2,LOW);}
        else if(initR<30){currentPosition='F'; digitalWrite(POS1,LOW);  digitalWrite(POS2,HIGH);}
        }
        Serial.print("Direction is:"); Serial.println(rotionDirection ); 
     Serial.print("initL is:"); Serial.println(initL); 
     Serial.print("RangeL is:"); Serial.println(rangeL); 
     Serial.print("RangeR is:"); Serial.println(rangeR ); 
    //++++++++++++++++++++++++++++++++++++++++++++++++++
         Serial.println("In Lerarn");
         obIndex=(obIndex+1)%4; 
         move_Stage=STAGE2;
               msg="";
               if (Serial.available())  {
                 msg="";
                 while(Serial.available()){
                       c=Serial.read();
                       msg=msg+c;
  
                      } 
                  Serial.print("Old msgs is:"); Serial.println(msg); 
  
                }
                 delayTime=millis();
         
    break;
    case STAGE2:/// learn obesticals and store in memory
       
         // obstacles[obIndex]="NNN";
             
                         // msg="";
               if (Serial.available() && (millis()-delayTime<=LEARN_TIME))  {
                 msg="";
                 while(Serial.available() ){
                       c=Serial.read();
                       msg=msg+c;
  
                      } 
                 
   Serial.print("Final Learn is :"); Serial.println(msg);
                }
                  Serial.print("position  is :"); Serial.println(currentPosition); 
               if(millis()-delayTime>=LEARN_TIME){move_Stage=STAGE3;  /*setSteering(STRAIGHT_STEERING);*/ delayTime=millis();}
               // FillObAndUturnState (msg,obIndex);
          /*
          obstacles[1]="NNN";  InverseObstacles[1]=inverseObs(obstacles[1]);
          obstacles[2]="FFF";  InverseObstacles[2]=inverseObs(obstacles[2]);
          obstacles[3]="NNN";  InverseObstacles[3]=inverseObs(obstacles[3]);
          obstacles[0]="FFF";  InverseObstacles[0]=inverseObs(obstacles[0]);

          */

       
          
        
    break;
    case STAGE3://just waite untel stearing is straight
          Serial.print("Final Learn is :"); Serial.println(msg);
          FillObAndUturnState (msg,obIndex);
          obstacles[1]="NNN";  InverseObstacles[1]=inverseObs(obstacles[1]);
          obstacles[2]="NNN";  InverseObstacles[2]=inverseObs(obstacles[2]);
          obstacles[3]="NNN";  InverseObstacles[3]=inverseObs(obstacles[3]);
          obstacles[0]="NNN";  InverseObstacles[0]=inverseObs(obstacles[0]);
          if(obIndex==0){Serial.println("All Ob learned : "); 
          Serial.print("1:>> ");  Serial.println( obstacles[1]);
           Serial.print("2:>> ");Serial.println( obstacles[2]);
           Serial.print("3:>> ");Serial.println( obstacles[3]);
           Serial.print("0:>> ");Serial.println( obstacles[0]);
           // UTurnState=checkUturn(); 
               UTurnState=true;
            Serial.print(""); Serial.println("");
          
          }
         if(millis()-delayTime>=TIME_PRELEARN){move_Stage=STAGE4;}
    break;
    case STAGE4:
         Serial.println("Finish learn and go to corner to turn");
         motion_mode=TURNE;
         move_Stage=STAGE1;
       
        
    break;
 
    }
  
  }



void turnCalculatons(){
     
    if((rounCounter-1)==0){
      readDistance(LEFT_RIGHT); 
      initR=rangeR;
      initL=rangeL;
        Serial.print(" rotionDirection ");Serial.println( rotionDirection);
      if(rotionDirection==CLOCKWIZE){
        
        if(initL>=1 && initL<=26){currentPosition='F';}
        else if(initL>=41 && initL<=48){currentPosition='C';}
        else if(initL>=60 && initL<=79){currentPosition='N';}
        } 
      else if(rotionDirection==ANTI_CLOCKWIZE){
       Serial.print(" initR: ");Serial.println(initR);
        if(initR>=60){currentPosition='N';}
        else if(initR>=40 && initR<60){currentPosition='C';}
        else if(initR<30){currentPosition='F';}
        }
    }

      if(obstacles[rounCounter%4].charAt(0)=='F'){nextPosition='F'; }
      else{nextPosition='N';}
       
         Serial.print(" rounCounter-1 : ");Serial.println( rounCounter-1);
       Serial.print("Memory="); Serial.print(obstacles[rounCounter%4]);
    //   Serial.print("Iint Range L: ");Serial.println(initL);

        Serial.print("currentPosition: ");Serial.println(currentPosition);
        Serial.print("Next Position: ");Serial.println(nextPosition);
         

// determine first stage of turn in corner     delayTime;        DO_TURN 
      if(currentPosition=='N'  &&  nextPosition=='N'){if(rotionDirection==ANTI_CLOCKWIZE){ encoderTurn=950;}else{encoderTurn=950;}
                                                      turne_Stages=DO_TURN; delayTime=millis(); }
      
      else if(currentPosition=='N'  &&  nextPosition=='F'){turne_Stages=TURN_STRAIGHT1; turnSteering(steeringAngle,STRAIGHT_STEERING,10); steeringAngle=STRAIGHT_STEERING;
                                                           if(rotionDirection==ANTI_CLOCKWIZE){ stright1PID=65; stright2PID=15; encoderTurn=1150;} 
                                                           else{setpoint = rangeL-3; stright1PID=65;  stright2PID=15; encoderTurn=1150;} }
      else if(currentPosition=='F'  &&  nextPosition=='N'){ if(rotionDirection==ANTI_CLOCKWIZE){encoderTurn=950;}else{encoderTurn=950;}
                                                        turne_Stages=DO_TURN;  delayTime=millis(); stright2PID=73;}////
      else if(currentPosition=='F'  &&  nextPosition=='F'){turne_Stages=TURN_STRAIGHT1; turnSteering(steeringAngle,STRAIGHT_STEERING,10); steeringAngle=STRAIGHT_STEERING;
                                                           if(rotionDirection==ANTI_CLOCKWIZE){setpoint = rangeR; stright1PID=rangeR;  stright2PID=15;encoderTurn=1150;} 
                                                           else{setpoint = rangeL; stright1PID=rangeL;  stright2PID=15;encoderTurn=1150;}}
      else if(currentPosition=='C'  &&  nextPosition=='N'){if(rotionDirection==ANTI_CLOCKWIZE){encoderTurn=950;}else{encoderTurn=950;}
                                                           turne_Stages=DO_TURN;  stright2PID=73;  delayTime=millis(); }  
      else if(currentPosition=='C'  &&  nextPosition=='F'){
                                                            turne_Stages=TURN_STRAIGHT1; turnSteering(steeringAngle,STRAIGHT_STEERING,10); steeringAngle=STRAIGHT_STEERING; 
                                                               if(rotionDirection==ANTI_CLOCKWIZE){setpoint = rangeR; stright1PID=rangeR;  stright2PID=15;encoderTurn=1150;}
                                                              else{setpoint = rangeL; stright1PID=rangeL;  stright2PID=15; encoderTurn=1150;}}
      
     if(rounCounter<4){stop(); delay(250);}
      initAngle=currentAngle; 
      Fact=1;
      if(rotionDirection==CLOCKWIZE){Fact=-1;}
     //2020202020202020202020202020202020202020202020202202020202020202002020202020202020202020202020202020202020202020202020202020202020202022020202020202
       
      if(afterUTurn && rounCounter>=8){nxtAng =nextAngle+((rounCounter-8)*Fact*90);}
      else{ nxtAng=angle0+((rounCounter)*Fact*90);}
      Serial.print("currenr Angle=");Serial.println(currentAngle);
      Serial.print("Next angle");Serial.println(nxtAng);
      encoder_count=0;

    encoder_count=0; 
  //motion_mode=STOP;  rounCounter

  }





//function to turn in corner 
void turnInCorner(){
    //readDistance(ALL);
    switch(turne_Stages){
      
      case TURN_STRAIGHT1:
 
      

            setSteering(steeringPID);
            forward(SPEED2);
            if(rangeF<37/* add  incoder condition  */){Serial.println("TURN STAGE");
              turne_Stages=DO_TURN; encoder_count=0;  delayTime=millis();}
          
      break;
      
      case TURN_STRAIGHT2:  
            Serial.println("IN STRIGHT 2");
           if((rotionDirection==ANTI_CLOCKWIZE && rangeL<100) && currentPosition=='N'){  turnSteering(steeringPID,STRAIGHT_STEERING,10);  CurrentSteeringAng=STRAIGHT_STEERING; stop(); delay(100); turne_Stages=CHECK; encoder_count=0; Serial.println("FINISH  Stright2 N in TURN STAGE");/*motion_mode=STOP;*/} 
           else if((rotionDirection==CLOCKWIZE && rangeR<100)&& currentPosition=='N'){   turnSteering(steeringPID,STRAIGHT_STEERING,10); CurrentSteeringAng=STRAIGHT_STEERING; stop(); delay(100); turne_Stages=CHECK; encoder_count=0; Serial.println("FINISH  Stright2 in  N  TURN STAGE");/*motion_mode=STOP;*/} 
           else if((rotionDirection==ANTI_CLOCKWIZE && rangeL<100) &&  encoder_count>550){   turnSteering(steeringPID,STRAIGHT_STEERING,10); CurrentSteeringAng=STRAIGHT_STEERING; stop(); delay(100); turne_Stages=CHECK; encoder_count=0; Serial.println("FINISH  Stright2 in TURN STAGE = ");/* Serial.println(setpoint);  motion_mode=STOP;*/}
          else if((rotionDirection==CLOCKWIZE && rangeR<100)&&    encoder_count>550){   turnSteering(steeringPID,STRAIGHT_STEERING,10); CurrentSteeringAng=STRAIGHT_STEERING; stop(); delay(100); turne_Stages=CHECK; encoder_count=0; Serial.println("FINISH  Stright2 in TURN STAGE=");/* Serial.println(setpoint); motion_mode=STOP;*/}
          else{
            debugUltrasonic();
           setSteering(steeringPID);
           forward(SLOW_SPEED4);
           }
          // debugUltrasonic(); 
          // setpoint=stright2PID; 
          
           
          // setSteering(STRAIGHT_STEERING); 
      //  turnSteering(steeringAngle);

             
      break;
      case DO_TURN:

           if(printAngles){
            Serial.println("_____________________________________________________");
              Serial.print("currentAngle="); Serial.println(currentAngle);
              Serial.print("nxtAng="); Serial.println(nxtAng);
               Serial.println("_____________________________________________________");
              printAngles=false;
            }
            complementaryMode=false;
            forward(SLOW_SPEED2);
              if(rotionDirection==ANTI_CLOCKWIZE  && nextPosition=='N' && steeringAngle != (STRAIGHT_STEERING-32)){forward(TURN_SPEED_L2);  turnSteering(steeringAngle,STRAIGHT_STEERING-32,10);  steeringAngle=STRAIGHT_STEERING-32;  delayTime=millis();} 
             else if(rotionDirection==CLOCKWIZE  && nextPosition=='N' && steeringAngle!=(STRAIGHT_STEERING+32)){forward(TURN_SPEED_L2);     turnSteering(steeringAngle,STRAIGHT_STEERING+32,10);steeringAngle=STRAIGHT_STEERING+32; delayTime=millis();} 
           
            else if(rotionDirection==ANTI_CLOCKWIZE  && nextPosition=='F' && steeringAngle != (STRAIGHT_STEERING-37)){forward(SLOW_SPEED3); steeringAngle=STRAIGHT_STEERING-37;  turnSteering(STRAIGHT_STEERING,steeringAngle,10);delayTime=millis();} 
            else if(rotionDirection==CLOCKWIZE  && nextPosition=='F' && steeringAngle!=(STRAIGHT_STEERING+37)){forward(SLOW_SPEED3);steeringAngle=STRAIGHT_STEERING+37;  turnSteering(STRAIGHT_STEERING,steeringAngle,10);delayTime=millis();} 
              readAngle();
             if(  abs(currentAngle-nxtAng)> 3  &&   encoder_count<encoderTurn && (millis()-delayTime)<=5000 )
             {
              //forward(SLOW_SPEED3);
            forward(SLOW_SPEED2); // if(encoder_count<(encoderTurn*0.7)){forward(SLOW_SPEED2);} else {forward(TURN_SPEED_L2);}
                   }

         else{  
              printAngles=true;
              //if(rounCounter==3){motion_mode=STOP; }//(millis()-delayTime)
              Serial.println("Finish turn ...........................");
             //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
              Serial.print("(millis()-delayTime) ="); Serial.println((millis()-delayTime));
             Serial.print("Turn Encoder count="); Serial.println(encoder_count);
              Serial.println("_____________________________________________________");
             Serial.print("currentAngle="); Serial.println(currentAngle);
              Serial.print("nxtAng="); Serial.println(nxtAng);
               Serial.println("_____________________________________________________");
             stop();// steering.write(STRAIGHT_STEERING); delay(300);
             turne_Stages=CHICK_SHIFT;
             turnSteering(steeringAngle,STRAIGHT_STEERING,10); //steeringAngle=STRAIGHT_STEERING;
             readDistanseFrom=LEFT_RIGHT;
             
              if(rotionDirection==ANTI_CLOCKWIZE &&  nextPosition=='N'){if(rangeR>=70 && rangeR<=74){setpoint = rangeR; } else {setpoint =73;}setpoint = rangeR; setpoint = 73;  }
              else if(rotionDirection==CLOCKWIZE &&  nextPosition=='N') {if(rangeL>=70 && rangeL<=74){setpoint = rangeL; } else {setpoint =73;} setpoint = rangeL;setpoint = 73; }
              
              if(rotionDirection==ANTI_CLOCKWIZE &&  nextPosition=='F'){setpoint = 16;  }

              
              else if(rotionDirection==CLOCKWIZE &&  nextPosition=='F') {setpoint = 16;}
              setpoint=stright2PID;
             
             encoder_count=0;
             //if(rounCounter==1){ motion_mode=STOP;}

                }
            
      break;
      /*
     shift_Stages=BRE_SHIFT;
            
  
       */
      case CHICK_SHIFT:
             
              Serial.println ("Check Siftttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttt");
              Serial.print("currentPosition "); Serial.println(currentPosition);
              Serial.print("nextPosition "); Serial.println(nextPosition);
              Serial.print("rangeR"); Serial.println(rangeR);
              Serial.print("rangeL"); Serial.println(rangeL);
             if(rotionDirection==ANTI_CLOCKWIZE  && (abs((int)rangeR-73)>2) && nextPosition=='N' && (currentPosition=='F' ) )
                  { shiftCM=constrain(abs((int)rangeR-73), 2,15); if(rangeR > 73){ turne_Stages=SHIFT_R;} else{ turne_Stages=SHIFT_L;}  shift_Stages=BRE_SHIFT;}
             else if(rotionDirection==ANTI_CLOCKWIZE  && (abs((int)rangeR-16)>3) && nextPosition=='F' && (currentPosition=='N' ) )
                  {/* shiftCM=constrain(abs((int)rangeR-16), 2,15); if(rangeR > 16){ turne_Stages=SHIFT_R;} else{ turne_Stages=SHIFT_L;}  shift_Stages=BRE_SHIFT; */turne_Stages=TURN_STRAIGHT2;}
             else if(rotionDirection==ANTI_CLOCKWIZE  && (abs((int)rangeR-16)>3) && nextPosition=='F' && currentPosition=='F' )
                  { shiftCM=constrain(abs((int)rangeR-16), 2,15); if(rangeR > 16){ turne_Stages=SHIFT_R;} else{ turne_Stages=SHIFT_L;}  shift_Stages=BRE_SHIFT;}

                   
             else if(rotionDirection==CLOCKWIZE  &&  ( abs((int)rangeL-73)>2) && nextPosition=='N' &&( currentPosition=='F' ))
                  {shiftCM=constrain(abs((int)rangeL-73), 2,15); if(rangeL > 73){ turne_Stages=SHIFT_L;} else{ turne_Stages=SHIFT_R;}  shift_Stages=BRE_SHIFT;}



             else if(rotionDirection==CLOCKWIZE  && (abs((int)rangeL-16)>3) && nextPosition=='F' && (currentPosition=='N' ) )
                  { /*shiftCM=constrain(abs((int)rangeL-16), 2,15); if(rangeL > 16){ turne_Stages=SHIFT_L;} else{ turne_Stages=SHIFT_R;}  shift_Stages=BRE_SHIFT;*/ turne_Stages=TURN_STRAIGHT2;}

                  
             else if(rotionDirection==CLOCKWIZE  &&  ( abs((int)rangeL-16)>3) && nextPosition=='F' && currentPosition=='F' )
                  {shiftCM=constrain(abs((int)rangeL-16), 2,15); if(rangeL > 16){ turne_Stages=SHIFT_L;} else{ turne_Stages=SHIFT_R;}  shift_Stages=BRE_SHIFT;}

             else{ turne_Stages=TURN_STRAIGHT2;}
           //  motion_mode=STOP;
             
      break;//shifLeft(    
       case SHIFT_L:
              shifLeftCm(shiftCM);
             if( shift_Stages==FINISH){turne_Stages=TURN_STRAIGHT2;}
             
             
      break;
      case SHIFT_R:
            shifRightCm(shiftCM);
            if( shift_Stages==FINISH){turne_Stages=TURN_STRAIGHT2;}
      break;

      case CHECK:
           Serial.println("IN CHECK");
            rangeB=readBackDistance(5);
           // Serial.print("Back Distance"); Serial.println(rangeB);
           if(rangeB<85){encoderReviece=100; turne_Stages=FORWARD2; encoder_count=0; }
           else if(rangeB>88){encoderReviece=150; turne_Stages=REVERSE2;encoder_count=0;}
           else {turne_Stages=FINISH_TURN; }
          
           
      break;
      
      case REVERSE2:
              rangeB=readBackDistance(5);
            Serial.print("in revirse"); Serial.println(rangeB);
           setSteering(STRAIGHT_STEERING);
           reverse(SLOW_SPEED);
           if(rangeB<88 || encoder_count>=encoderReviece){stop(); turne_Stages=FINISH_TURN;}
      break;
      case FORWARD2:
           rangeB=readBackDistance(5);
         //  Serial.print("in Forward"); Serial.println(rangeB);
           setSteering(STRAIGHT_STEERING);
           forward(SLOW_SPEED);
           if(rangeB>85/*encoder_count>=encoderReviece*/){stop(); turne_Stages=FINISH_TURN;}
      break;
      
      case FINISH_TURN:
        //  Serial.print("Final Back Distance"); Serial.println(rangeB);
          obstacle_Stage=FIRST;
          motion_mode=BTWEEN_OB;
         // stop();
          //  rangeB=readBackDistance(5);
           //Serial.print("Back Distance: "); Serial.println(rangeB);
      //   motion_mode=STOP;
          if(rounCounter==5 || rounCounter==9){

             obIndex=rounCounter%4;
            Serial.println("we must between obbbbbbbbbbbbbbbbbbbbbbbbbbbbb");
            Serial.print("rounCounter : "); Serial.println(rounCounter);
            Serial.print("currentPosition : "); Serial.println(currentPosition);
            Serial.print("I am in  : "); Serial.println( obstacles[rounCounter%4].charAt(0));
              Serial.print("Memory at this   : "); Serial.println( obstacles[obIndex]);
          //  motion_mode=STOP;// FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFf
            }
      break;
 
    }
  }

  // turn in corner and ready to ... between ob.s
void  doTurn2(){
  

  
    switch(move_Stage){
      case STAGE1:
      
       rounCounter++;
       turnCalculatons();
        if(rotionDirection==ANTI_CLOCKWIZE ){ readDistanseFrom=RIGHT_FORWARD;}else {readDistanseFrom=LIFT_FORWARD;}
       readDistance(readDistanseFrom);
       move_Stage=STAGE2;
     //  if(rounCounter==2){ motion_mode=STOP;}
      //readAngle();  motion_mode=STOP;
     
      break;
      case STAGE2:
       // if( nextPosition=='N' &&  currentPosition=='F' &&  readForwardDistance(3)<95){}
        move_Stage=STAGE3;
           delayTime=millis();   
      break;
      case STAGE3:
      // between obest.....
        turnInCorner();
      break;
     
 
    }
  
  
  
  }

void doParking2(){

     switch (Parking_stages)  
  {
  case CLEAR_SERIAL:
   digitalWrite(POS1,HIGH);
   digitalWrite(POS2,HIGH);
                    if (Serial.available())  {
                 msg="";
                 while(Serial.available()){
                       c=Serial.read();
                       msg=msg+c;
  
                      } 
                  Serial.print("buffer hase text : "); Serial.println(msg); Parking_stages=RCIVE_SERIAL; parkingState=true; setpoint=10.0;
  
                }

  break;
  case RCIVE_SERIAL:      //  cameraDist     steeringPID
                 if (Serial.available())  {
                 msg="";
                 while(Serial.available()){
                       c=Serial.read();
                       msg=msg+c;
  
                      } 
                  Serial.print("buffer hase text : ");
                  if(msg.charAt(0)=='-'){msg=msg.substring(0,3);} else {msg=msg.substring(0,2);}
                    Serial.println(msg); 
                   cameraDist=msg.toDouble();  //Serial.print("Double number is "); Serial.println(cameraDist);
                   
                   
                }
                setSteering(steeringPID);
                forward(SLOW_SPEED);
                if(readForwardDistance(5) <17 ){stop();Parking_stages=GO_PID; setpoint=7.0;  parkingState=false; parkingTime=millis();}
                

  break;
  case GO_PID:  
           Serial.println("GO_PID");  
         

           readDistance(ALL); 
           rotionDirection=ANTI_CLOCKWIZE ;
           if(rangeF>2 && (millis()-parkingTime)<=4000){
                setSteering(steeringPID);
                forward(SLOW_SPEED);}
           else{stop(); Parking_stages=FINSH_PARKING; }
  break;
  case FINSH_PARKING:
         motion_mode=STOP;
         Serial.println("FINSH_PARKING");
  break;
  
  }
}
void doLevel2(){
 // if(pidState){steering.write(steeringPID); }  
  switch (motion_mode)  
  {
  case STANDBY:

  break;
  case FORWARD:
      if(!isStarted){isStarted=true; runTime2=millis();}
     doForward2(); 
   
   
  break;
  case ENTER_CORNER:
       doEnterCorner();
  break;
  case LEARN:
       OB_Learn();
  case CORNER:
    //goToCorner();
  break;
  case TURNE:
   doTurn2();
  break;
  case BTWEEN_OB:
    betweenOBbstacles();
  break;
    case BTWEEN_OB8:
   
  break;
  case PARKING:
    doParking2();

  break;
  case STOP:
    stop();
    isStarted=false ; Serial.print("Time is : ");Serial.println((millis()-runTime2)/1000);            
   delay(1000);
   readAngle();
  // debugRotation();
    ESP.restart();    
  break;

  }
  }
  
