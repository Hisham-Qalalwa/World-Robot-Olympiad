#define ALL 0
#define LEFT_RIGHT 1
#define LIFT_FORWARD 2
#define RIGHT_FORWARD 3


#define TIMEOUT 16500
HardwareSerial ultraSensor(1); 
unsigned int rangeR,rangeL,rangeF;
unsigned int initR,initL,initF,rangeB;
unsigned int prevRangeR,prevRangeL,prevRangeF;

unsigned long UltraSonicUpdateTime;
float distance;
unsigned long period;
void initUltrasonic(){
  
  
pinMode(ECHO_R,INPUT);
pinMode(TRIG_R,OUTPUT);
pinMode(ECHO_L,INPUT);
pinMode(TRIG_L,OUTPUT);
pinMode(ECHO_F,INPUT);
pinMode(TRIG_F,OUTPUT);
pinMode(ECHO_B,INPUT);
pinMode(TRIG_B,OUTPUT);


}

int findeDist(byte E,byte T){
int d;
digitalWrite(T,LOW);
delayMicroseconds(5);
digitalWrite(T,HIGH);
delayMicroseconds(10);
digitalWrite(T,LOW);
period=pulseIn(E,HIGH,17492);
distance=float(period)/58.8;
d=(int)distance; 
d=constrain(d, 0, 301);
if(d==0){d=300;}
return(d);
}

void readUltrasonic()
{
  prevRangeR=rangeR;
  prevRangeL=rangeL;
  prevRangeF=rangeF;
  rangeR=findeDist(ECHO_R,TRIG_R);
  delay(3);
  rangeF=findeDist(ECHO_F,TRIG_F);
  delay(2);
  rangeL=findeDist(ECHO_L,TRIG_L);
  delay(2);
 // rangeF=findeDist(ECHO_F,TRIG_F);
  
}

unsigned int readForwardDistance (int times){
  unsigned int dd=0;
  for(int i=0;i<times;i++){
  dd=dd+findeDist(ECHO_F,TRIG_F);delay(1);
  }
  rangeF=dd/times;
   return rangeF;
  }

unsigned int readBackDistance(int times){
  unsigned int dd=0;
  for(int i=0;i<times;i++){
   delay(1); dd=dd+findeDist(ECHO_B,TRIG_B);delay(1);
  }
  rangeB=dd/times;
   return rangeB;
  }
 
void readDistance(int readCase){  
  prevRangeR=rangeR;
  prevRangeL=rangeL;
  prevRangeF=rangeF;

  switch (readCase){
    case  ALL:
      rangeR=findeDist(ECHO_R,TRIG_R);
      rangeF=findeDist(ECHO_F,TRIG_F);
      rangeL=findeDist(ECHO_L,TRIG_L);
      delay(1);
    break; 
    case LEFT_RIGHT :
       delay(5);
          rangeL=findeDist(ECHO_L,TRIG_L);
     
      delay(5);
          rangeR=findeDist(ECHO_R,TRIG_R);
          
      delay(5);
      
    break; 
    case LIFT_FORWARD:
      rangeF=findeDist(ECHO_F,TRIG_F);
      delay(2);
      rangeL=findeDist(ECHO_L,TRIG_L);
      delay(2);
    break;
    case RIGHT_FORWARD:
      rangeR=findeDist(ECHO_R,TRIG_R);
      delay(2);
      rangeF=findeDist(ECHO_F,TRIG_F);
      delay(2);
    break; 
    }
  
  }

void debugUltrasonicFrom(int readCase){
           Serial.println("Read From : ");
    switch (readCase){
    case  ALL:
             Serial.println("ALL");
    break; 
    case LEFT_RIGHT :
        Serial.println("LEFT_RIGHT");
    break; 
    case LIFT_FORWARD:
         Serial.println("LIFT_FORWARD");
    break;
    case RIGHT_FORWARD:
           Serial.println("RIGHT_FORWARD");
    break; 
    }
  
  Serial.print("R: ");Serial.print(rangeR);
  Serial.print("  L: ");Serial.print(rangeL);
  Serial.print("  F: ");Serial.print(rangeF);
  Serial.println();}


  void debugUltrasonic(){
   
  
  Serial.print("R: ");Serial.print(rangeR);
  Serial.print("  L: ");Serial.print(rangeL);
  Serial.print("  F: ");Serial.print(rangeF);
  Serial.println();}

/*
char c;
String readString;

void initUltrasonic(){
  ultraSensor.begin(9600,SERIAL_8N1,RXD2,TXD2);
 
  rangeR=rangeL=rangeF=0;
  UltraSonicUpdateTime=0;
  
   }
void readUltrasonic(){
    readString="";
    while (ultraSensor.available()) {
    c = ultraSensor.read();
    readString += c;
    Serial.println(c);
  }
  if (readString.length() > 0) {Serial.print(readString);}
  }
*/
