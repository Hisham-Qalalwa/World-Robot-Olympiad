

char c;
String msg;
void initSerial() {

 Serial.begin(115200);

}

void readSerial() {
  msg="";
  if (Serial.available())  {
    msg="";
      while(Serial.available()){
    c=Serial.read();
    msg=msg+c;
  
  } 
  Serial.print("msg is:"); Serial.println(msg); 
  }

}
