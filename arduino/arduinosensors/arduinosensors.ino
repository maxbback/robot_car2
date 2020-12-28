#include <Servo.h>


// Collect sensor information from
// Ultrasonic distance
// IR distance
// IR Bumper
// Minimum of calculation is done on arduino, this is left to consumer
// to colelct data as fast as possible
// we use 1 fixed ultrasonic and ir sensor for front detection
// we use sg 90 servo to loop 180 degree getting 360 data colection
// We loop our sensors in sequence always starting with bumper
// if bumper reacts we sent data directly and continue colelct information for other sensors
// 
// Each sequence we also collect angle that reflects relation to front
// 90 degree is front
// due to limitations in sg90 we never physical reach 180 degree
// At 0 degree we collect left and right side 90 degree from front
// at 90 degree we collect front and back
// at 180 we rotated so sensor left now collect 87 degree on right side
// and right sensor collect -3 degree on left side
//
// We can send up to 10 differnt messages types
// 0 send bumper status 0 bumped, 1 not bumped
// 1 front US
// 2 front IR
// 3 scanner US
// 4 scanner IR
//
// Message format
// message type:angle:distance:speed
//
// bumper sends a message when something change
//




// Temporary global variables
long duration;
int usDistance;
int usSpeed;
int tmpDistance;
// curent timestamp
long currentTime = millis();

// Max distance from US if ouside we consider it invalid
int maxUsDistance = 400;

unsigned long scannerTime[180];
int scannerUsMeasure[360]; //distance
unsigned long frontUsMeasure[3]; // time,distance,speed
int frontBumped = 0;

// Pin configuration for scanner
int scannerUsTriggerPin[2] = {11,5};
int scannerUsEchoPin[2] = {10,4};

// Pin configuration front sensors
int frontUsTriggerPin = 9;
int frontUsEchoPin = 8;
int frontBumperPin = 2;




// Servo
Servo scannerServo;
int scannerServoAngle = 0;
int scannerServoAngleMax = 177;
int scannerServoAngleMin = 3;
int scannerServoPin = 3;


// Enable or disable sensores 0 == disabled
int scannerUs = 1;
int scannerUsNr = 0;
int frontUs = 1;
int bumperFront = 1;
int scannerServoEnabled = 1;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  for ( int i = 0; i < 2; i = i + 1) {
    pinMode(scannerUsTriggerPin[i], OUTPUT);
    pinMode(scannerUsEchoPin[i], INPUT);
  }
  if (frontUs == 1) {
    pinMode(frontUsTriggerPin, OUTPUT);
    pinMode(frontUsEchoPin, INPUT);
  }
  if (scannerServoEnabled == 1) {
    scannerServo.attach(scannerServoPin); // D9
    scannerServo.write(scannerServoAngle);
  }
  // Initite scanner measure array
  for (int i = 0; i < 180; i = i + 1) {
    // time, distance, speed
    scannerUsMeasure[i] = maxUsDistance;
  }
  // Initiate front measure array
  frontUsMeasure[0] = currentTime;
  frontUsMeasure[1] = -1;
  frontUsMeasure[2] = -1;
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(analogRead(A0));
  
  String s = "";
  int toClose = 0;
  int bumped = 0;
  int speed = 0;


  
  for(scannerServoAngle = scannerServoAngleMin; scannerServoAngle < scannerServoAngleMax; scannerServoAngle = scannerServoAngle + 2)  
  {                                  
  // Collect bumper
  if (bumperFront == 1) {
    bumped = getBumper(frontBumperPin);
    if (bumped != frontBumped) {
      //type:angle:distance:speed
      s = "0:90:" + String(bumped) + ":-1";
      Serial.println(s);
      frontBumped=bumped;      
    }
  }
    s="1";
    scannerServo.write(scannerServoAngle);
    collectData(scannerServoAngle);            
    delay(30);                   
  } 
  // now scan back from 180 to 0 degrees
  for(scannerServoAngle = scannerServoAngleMax; scannerServoAngle > scannerServoAngleMin; scannerServoAngle = scannerServoAngle - 2)    
  {                                
  // Collect bumper
  if (bumperFront == 1) {
    bumped = getBumper(frontBumperPin);
    if (bumped != frontBumped) {
      //type:angle:distance:speed
      s = "0:90:" + String(bumped) + ":-1";
      Serial.println(s);
      frontBumped=bumped;      
    }
  }
    s="1";
    scannerServo.write(scannerServoAngle);          
    collectData(scannerServoAngle);            
    delay(30);       
  } 

  
}

int collectData(int angle) {
  String s;
  int sensorAngle;
  int j;

  // time,distance,speed
  usDistance = getUsDistance(frontUsTriggerPin,frontUsEchoPin);
  currentTime = millis();
  if (usDistance >= maxUsDistance) {
    usDistance = frontUsMeasure[1];
    usSpeed = frontUsMeasure[2];
  } else {
    usSpeed = getSpeed(frontUsMeasure[0],frontUsMeasure[1],usDistance,maxUsDistance);
  }
  frontUsMeasure[0] = currentTime;
  frontUsMeasure[1] = usDistance;
  frontUsMeasure[2] = usSpeed;

  
  //type:angle:distance:speed
  s="1:90:" + String(usDistance) + ":" + String(usSpeed);
  Serial.println(s);

  /*
   //return 0;
   if (scannerUsNr == 0) {
    scannerUsNr = 1;
   } else {
    scannerUsNr = 0;
   }
   j=scannerUsNr;
      sensorAngle = angle + scannerServoAngleMax * j;
      usDistance = getUsDistance(scannerUsTriggerPin[j],scannerUsEchoPin[j]);
      currentTime = millis();
      // us
      if (usDistance >= maxUsDistance) {
        usDistance = scannerUsMeasure[sensorAngle];
      }
      
      scannerUsMeasure[sensorAngle] = usDistance;

      //type:angle:distance:speed
      s="3:" + String(sensorAngle) + ":" + String(usDistance) + ":-1" ;
      Serial.println(s);

 */     

  for (int j = 0; j < 1; j = j + 1) {
      Serial.println("fron collected " + String(angle) + " " + String(j));
      Serial.println("trig " + String(scannerUsTriggerPin[j]) + " echo " + String(scannerUsEchoPin[j]));
      sensorAngle = angle + scannerServoAngleMax * j;
      usDistance = getUsDistance(scannerUsTriggerPin[j],scannerUsEchoPin[j]);
      currentTime = millis();
      // us
      if (usDistance >= maxUsDistance) {
        usDistance = scannerUsMeasure[sensorAngle];
      }
      
      scannerUsMeasure[sensorAngle] = usDistance;

      //type:angle:distance:speed
      s="3:" + String(angle  + scannerServoAngleMax * j) + ":" + String(usDistance) + ":-1" ;
      Serial.println(s);

  }

scannerTime[angle]=currentTime;
  //Serial.println("fron collected");
  return 0;
}

int getSpeed(int lastTime, int lastDistance, int distance, int maxDistance) {
  int deltaDistance = 0;
  int speed = -1;
  //delay(100);

  if (distance == maxDistance) {
    speed = -1;
  } else {
    if (currentTime < lastTime) {
      // timer has reset, restarting
      speed = -1;
    } else {
      deltaDistance = abs(lastDistance - distance)*1000;
      //Serial.println(deltaDistance);
      //Serial.println(currentTime - usLastTimeStamp[sensor]);
      if (deltaDistance > 0) {       
        speed = deltaDistance / (currentTime - lastTime);
      } else {
        speed = 0;
      }
    }
  }
    
  return speed; // Distance in CM/milisecond
  
}

int getBumper(int pin) {
  return digitalRead(pin);
}

int getUsDistance(int trigPin, int echoPin) {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);  
  //Serial.println(duration);
  usDistance = duration * 0.034 / 2;
    
  return usDistance; // Distance in CM
}
