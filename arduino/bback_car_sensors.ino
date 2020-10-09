#include <SharpIR.h>

long duration;
int distance;
int tmpDistance;
int maxUsDistance = 400;
long currentTime = millis();

int bumperPinArray[] = {9}; 
int trigPinArray[] = {5, 7, 3}; // F,L,R
int echoPinArray[] = {6, 8, 4};
int usLastDistance[] = {maxUsDistance, maxUsDistance, maxUsDistance};// F,L,R
long usLastTimeStamp[] = {currentTime,currentTime,currentTime}; // -1 indicates no previous time
double usSpeed[] = {-1,-1,-1}; // Current speed,-1 indicates unknowm speed 
int numberOfUs = 3;
int numberOfIr = 0;
int numberOfBumper = 0;


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
  String bs = "1"; // If bumped
  int toClose = 0;
  int bumped = 0;
  int speed = 0;
  for (int i = 0; i < numberOfBumper; i = i + 1) {
    bumped = getBumper(bumperPinArray[i]);
    s = s + ":" + String(bumped);
    bs = bs + ":" + String(bumped);
    if (bumped == 1) {
      toClose = 1;
    }
  }

  if (toClose == 1) { // Sent a emrgency stop
    bs = s;
    for (int i = 0; i < numberOfUs; i = i + 1) {
      bs = bs + ":0:0";
    }
    for (int i = 0; i < numberOfIr; i = i + 1) {
     bs = bs + ":0";
    }
    Serial.println(bs);
  }
  
  for (int i = 0; i < numberOfUs; i = i + 1) {
    int distance = getUsDistance(trigPinArray[i],echoPinArray[i]);
    currentTime = millis();
    if (distance >= maxUsDistance) {
      distance = usLastDistance[i];
    }
    speed = getUsSpeed(i,distance);
    usLastDistance[i] = distance;
    s = s + ":" + String(distance);
    s = s + ":" + String(speed);
  }
  
  for (int i = 0; i < numberOfIr; i = i + 1) {
    s = s + ":" + String(getIrDistance(i));
  }
  
  Serial.println(s);
}

int getIrDistance(int irNr) {
  int irvalues[25];
  int result = irSensorArray[irNr].distance();

  //int result = 0;
  //for (int i = 0; i < 25; i = i + 1) {
  //  irvalues[i] = analogRead(irNr);
  //}
   // Number of items in the array
  //int lt_length = sizeof(irvalues) / sizeof(irvalues[0]);
  // qsort - last parameter is a function pointer to the sort function
  //qsort(irvalues, lt_length, sizeof(irvalues[0]), sort_desc);
  
  //result = irvalues[13];
  return result;
}

int getUsSpeed(int sensor, int distance) {
  int deltaDistance = 0;
  int speed = -1;
  //delay(100);

  if (usLastDistance[sensor] == maxUsDistance) {
    speed = -1;
  } else {
    if (currentTime < usLastTimeStamp[sensor]) {
      // timer has reset, restarting
      speed = -1;
    } else {
      deltaDistance = abs(usLastDistance[sensor] - distance)*1000;
      //Serial.println(deltaDistance);
      //Serial.println(currentTime - usLastTimeStamp[sensor]);
      if (deltaDistance > 0) {       
        speed = deltaDistance / (currentTime - usLastTimeStamp[sensor]);
      } else {
        speed = 0;
      }
    }
  }
  usSpeed[sensor] = speed;
  
  usLastTimeStamp[sensor] = currentTime;
  //Serial.print(speed);
    
  return speed; // Distance in CM/milisecond
}

int getBumper(int pin) {
  return digitalRead(pin);
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


// qsort requires you to create a sort function
int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}
