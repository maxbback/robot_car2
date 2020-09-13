#include <SharpIR.h>

long duration;
int distance;
int tmpDistance;
int maxUcDistance = 400;

int trigPinArray[] = {5, 7, 3}; // F,L,R
int echoPinArray[] = {6, 8, 4};
int usLastDistance[] = {maxUcDistance, maxUcDistance, maxUcDistance}; // F,L,R
int numberOfUs = 3;
int numberOfIr = 3;

#define model 1080

SharpIR irSensorArray[] = {
  SharpIR(0, model),
  SharpIR(2, model),
  SharpIR(1, model)
};



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  for ( int i = 0; i < numberOfUs; i = i + 1) {
    pinMode(trigPinArray[i], OUTPUT);
    pinMode(echoPinArray[i], INPUT);
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(analogRead(A0));
  String s = "0";
  
  for (int i = 0; i < numberOfUs; i = i + 1) {
    int distance = getUsDistance(trigPinArray[i],echoPinArray[i]);
    if (distance >= maxUcDistance) {
      distance = usLastDistance[i];
    } 
    usLastDistance[i] = distance;
    s = s + ":" + String(distance);
  }
  for (int i = 0; i < numberOfIr; i = i + 1) {
    s = s + ":" + String(getIrDistance(i));
  }
  Serial.println(s);
  //delay(200);
}

int getIrDistance(int irNr) {
  // 10-80 cm
  int result = irSensorArray[irNr].distance();
  return result;
}

int getUsDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);  
  //Serial.println(duration);
  distance = duration * 0.034 / 2;
    
  return distance; // Distance in CM
}
