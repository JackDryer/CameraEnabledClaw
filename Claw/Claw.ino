#include <Servo.h>

String str;
int pos;
Servo myservo;
void setup() {
  Serial.begin(115200);
  myservo.attach(9); 
}
void loop() {
  if (Serial.available()>0)
  {// put your main code here, to run repeatedly:
  str = Serial.readStringUntil("\n");
  pos =str.toInt();
  if (pos>=32 && pos<=160){
    myservo.write(pos);
  }
  Serial.println(pos);
  //Serial.println("\r\n");
  }
}
