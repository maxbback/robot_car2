#include <Servo.h>
Servo servo;
int angle = 0;
void setup() {
  Serial.begin(115200);
  servo.attach(9); // D9
//  for (angle = servo.read(); angle > 0; angle--) {
    servo.write(angle);
   //delay(1000);
//  }
}
void loop() 
{ 
  long ct = millis();
  
 // scan from 0 to 180 degrees
  Serial.println("0");
  servo.write(0);               
  for(angle = 0; angle < 250; angle=angle+2)  
  {                                  
    servo.write(angle);               
    delay(20);                   
  }
  long nt = millis();
  Serial.println("hej");
  Serial.println(String(nt-ct));
 
  // now scan back from 180 to 0 degrees
  for(angle = 180; angle > 0; angle=angle-2)    
  {                                
    servo.write(angle);           
    delay(20);       
  }
  nt = millis();
  Serial.println("hej");
  Serial.println(String(nt-ct));
 
}
