///////////////////////////////////////////////////////////////
//////////////////////// ROS Robo Car /////////////////////////
///////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <Servo.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nodeHandleROBO;

std_msgs::UInt16 msgMode;
std_msgs::UInt16 msgEye;
std_msgs::UInt16 msgLine;


Servo myservo;

int minAngle = 700;
int maxAngle = 2150;
int rightDistance = 0, leftDistance = 0, middleDistance = 0;
int echoPin = A4;               
int trigPin = A5;               
int distance = 0;
int average = 0;
int stopDistance = 20;          //Change
int stopDistanceShort = 8;      //Change
int speedDistance = 100;        //Change
int backDistance = 20;          //Change
int i = 0;

#define SEA        3            //Servo Pin
#define ENA        5            
#define ENB        6            
#define IN1        7            
#define IN2        8            
#define IN3        9            
#define IN4       11            
#define LT_R      !digitalRead(10)
#define LT_M      !digitalRead(4)     
#define LT_L      !digitalRead(2)    

#define carSpeedMin       0     //Change
#define carSpeed        100     //Change
#define carSpeedturn    180     //Change
#define carSpeedMax     255     //Change

bool activeLineSensor = false;
bool activeEyeSensor = false;
bool modeIdenEye = false;
bool modeIdenLine = false;

long lastSlowup = 0;
long lastForward = 0;
long lastSlowdown = 0;
long lastBack = 0;
long lastLeft = 0;
long lastRight = 0;
long lastStop = 0;
long lastDistance = 0;
long lastSetSensorBack = 0;
long lastPublished = 0;

///////////////////////////////////////////////////////////////

int updateDistanceFast() {

  distance = 0;
  average = 0;

  digitalWrite(trigPin, LOW);     //0v
  delayMicroseconds(2);           //Emit 40 kHz sound
  digitalWrite(trigPin, HIGH);    //5v
  delayMicroseconds(10);          //for 10 microseconds
  digitalWrite(trigPin, LOW);   

  distance = pulseIn(echoPin, HIGH);  //detect a pulse 

  distance = distance / 74 / 2 * 2.54;  //Speed of sound 13511.81 inches/s
                                        //13511.81/10^6 inches per micro
                                        //0.01351181 inches per microsecond
                                        //74.0092341 microseconds per inch
                                        //dividing by 74 and dividing by 2 and multiple by 2.54 for cm
  
  return distance;
  
}  

///////////////////////////////////////////////////////////////

int updateDistance() {

  distance = 0;
  average = 0;

  for (int i = 0; i < 4; i++) {     //Build an Average

    digitalWrite(trigPin, LOW);     //0v
    delayMicroseconds(2);           //Emit 40 kHz sound
    digitalWrite(trigPin, HIGH);    //5v
    delayMicroseconds(10);          //for 10 microseconds
    digitalWrite(trigPin, LOW);   

    distance = pulseIn(echoPin, HIGH);  //detect a pulse 

    distance = distance / 74 / 2 * 2.54;  //Speed of sound 13511.81 inches/s
                                          //13511.81/10^6 inches per micro
    average += distance;                  //0.01351181 inches per microsecond
                                          //74.0092341 microseconds per inch
    delay(10);                             //dividing by 74 and dividing by 2 and multiple by 2.54 for cm
  }
  distance = average / 4;
  return distance;

}  

///////////////////////////////////////////////////////////////

void measureMiddleDistance () {

  if ((millis() - lastDistance > 1000)) {
    Serial.print("D: ");
    Serial.print(middleDistance);
    Serial.println(" cm");
    lastDistance = millis();
  }

}

///////////////////////////////////////////////////////////////

void slowup(){ 

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  if (i < carSpeedMax) {

    for(i = carSpeed; i <= carSpeedMax; i++){ 
      analogWrite(ENB,i);
      analogWrite(ENA,i);
      
      middleDistance = updateDistanceFast();

      if (middleDistance <= stopDistance) {
        break;
      }

      delay(5);
    }
  }

  if ((millis() - lastSlowup > 1000)) {
      Serial.println("Sup");
      lastSlowup = millis();
  }
}

void forward(){ 

  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  if ((millis() - lastForward > 1000)) {
      Serial.println("F");
      lastForward = millis();
  }
}

void slowdown() {

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  if (i > carSpeed) {

    for(i = carSpeedMax; i >= carSpeed; i--){ 
      analogWrite(ENB,i);
      analogWrite(ENA,i);

      middleDistance = updateDistanceFast();

      if (middleDistance <= stopDistance) {
        break;
      }

      delay(5);
    }
  }

  if ((millis() - lastSlowdown > 1000)) {
      Serial.println("Sdown");
      lastSlowdown = millis();
  }
}

void back() {
 
  analogWrite(ENA, carSpeedturn);
  analogWrite(ENB, carSpeedturn);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  if ((millis() - lastBack > 1000)) {
      Serial.println("B");
      lastBack = millis();
  }
}

void left() {
  analogWrite(ENA, carSpeedturn);
  analogWrite(ENB, carSpeedturn);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 

  if ((millis() - lastLeft > 1000)) {
      Serial.println("L");
      lastLeft = millis();
  }
}

void right() {
  analogWrite(ENA, carSpeedturn);
  analogWrite(ENB, carSpeedturn);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  if ((millis() - lastRight > 1000)) {
      Serial.println("R");
      lastRight = millis();
  }
}

void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);

  if ((millis() - lastStop > 1000)) {
    Serial.println("S");
    lastStop = millis();
  }
} 

///////////////////////////////////////////////////////////////

void eyeFunction () {

  if (modeIdenEye == false) {
    Serial.println("");
    Serial.println("#######################");
    Serial.println("####### EYE MODE ######");
    Serial.println("#######################");
    Serial.println("");

  modeIdenEye = true;
  modeIdenLine = false;
  }

  middleDistance = updateDistance();
  measureMiddleDistance();

  msgEye.data = middleDistance;

  if(middleDistance <= stopDistance) {      //Stop if something in the way
    stop();

    myservo.write(10);
    delay(500);
    rightDistance = updateDistance();

    myservo.write(180);
    delay(1000);
    leftDistance = updateDistance();

    myservo.write(90);
    delay(500);
    
    if(rightDistance > leftDistance) {
      right();
      delay(300 + random(150));
    }
    else if(rightDistance < leftDistance) {
      left();
      delay(300 + random(150));
    }
    else if((rightDistance <= backDistance) || (leftDistance <= backDistance)) {
      back();
      delay(150 + random(75));
    }
  }
  
  if(middleDistance > stopDistance){
    forward();
    delay(150 + random(75));
    
    while(middleDistance > speedDistance){
      slowup();
      middleDistance = updateDistance();
      measureMiddleDistance();
      if (middleDistance <= speedDistance) {
        slowdown();
        break;
      }
    }
  }
}

///////////////////////////////////////////////////////////////

void lineFunction () {

  if (modeIdenLine == false) {
    Serial.println("");
    Serial.println("#######################");
    Serial.println("###### LINE MODE ######");
    Serial.println("#######################");
    Serial.println("");

  modeIdenEye = false;
  modeIdenLine = true;
  }

  middleDistance = updateDistance();
  measureMiddleDistance();

  msgEye.data = middleDistance;

  if(middleDistance <= stopDistanceShort) {      //Stop if something in the way
    stop();
  }

  if(middleDistance >= stopDistanceShort) {      //Stop if something in the way
  
    if(LT_M){
      msgLine.data = LT_M;
      forward();
    }

    else if(LT_R) { 
      msgLine.data = LT_R;
      right();
      while(LT_R);                             
    }

    else if(LT_L) {
      msgLine.data = LT_L;
      left();
      while(LT_L);  
    }
  }
}

///////////////////////////////////////////////////////////////

void subscriberCallback(const std_msgs::UInt16& msgMode) {

  if (msgMode.data == 0) {
    activeEyeSensor = false;
    activeLineSensor = false;
    stop();
  }
  
  if (msgMode.data == 1) {
    activeEyeSensor = true;
    activeLineSensor = false;
  }

  if (msgMode.data == 2) {
    activeEyeSensor = false;
    activeLineSensor = true;
  }

}

///////////////////////////////////////////////////////////////

ros::Subscriber<std_msgs::UInt16> mode_subscriber("mode", &subscriberCallback);
ros::Publisher pub_eye("eye", &msgEye);
ros::Publisher pub_line("line", &msgLine);

///////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(9600);

  Serial.println("");
  Serial.println("ROBO!");

  myservo.attach(SEA,minAngle,maxAngle);
  myservo.write(90);

  nodeHandleROBO.initNode();
  nodeHandleROBO.subscribe(mode_subscriber);
  nodeHandleROBO.advertise(pub_eye);
  nodeHandleROBO.advertise(pub_line);

  pinMode(echoPin, INPUT);    
  pinMode(trigPin, OUTPUT);

  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);

  pinMode(LT_R,INPUT);
  pinMode(LT_M,INPUT);
  pinMode(LT_L,INPUT);

  digitalWrite(ENA, HIGH); 
  digitalWrite(ENB, HIGH);

  delay(1500);

}

///////////////////////////////////////////////////////////////

void loop() {

  if (activeEyeSensor == true) {
    pub_eye.publish(&msgEye);
    eyeFunction();

  }

  if (activeLineSensor == true) {
    pub_eye.publish(&msgEye);
    pub_line.publish(&msgLine);
    lineFunction();

  }

  nodeHandleROBO.spinOnce();

}

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////