/*  CODE WRITTEN BY JOSANU RARES IONUT for the project DriveJR 
 *  VERSION: 1.0
 *  LAST MODIFIED: 08.06.2018
 */
#include <Servo.h>

//----------FRONT UltraSonic (OFF)------
#define trigPinSL 53 //DIGITAL
#define echoPinSL 52 //DIGITAL
#define trigPinSR 48 //DIGITAL
#define echoPinSR 49 //DIGITAL
#define trigPinFRONT 50 //DIGITAL
#define echoPinFRONT 51 //DIGITAL
#define frontTimeout 20000 //timeout if out of range
//----------BACK UltraSonic-------
#define echoPinBACK 0 //DIGITAL
#define trigPinBACK 0//DIGITAL
#define echoPinBR 0//DIGITAL
#define trigPinBR 0//DIGITAL
#define echoPinBL 0//DIGITAL
#define trigPinBL 0//DIGITAL
//-----------PWM------------------
//-----------ACC & GYRO(OFF)-----------
#define INT_Acc 47 //DIGITAL
#define SCL_Acc A8//ANALOG
#define SDA_Acc A9//ANALOG
//--------ADDITIONAL SENSORS-----
#define tempSensor 46 //DIGITAL
#define encoderPin A0 //DIGITAL (OFF)


//--------MOTOR DRIVER-----------
#define MOTOR2_PIN1 6
#define MOTOR2_PIN2 9
#define MOTOR1_PIN1 3 
#define MOTOR1_PIN2 5 
int carSpeed = 0;
int dirServoPos = 60;
//--------COMMAND PINS NODEMCU--------

//--------------------SENSOR ENABLE------------------
bool SideSensor_R_ENABLED = true;
bool SideSensor_L_ENABLED = true;
bool FrontSensor_ENABLED = true;
//---------------------------------------------------

Servo directionServo;
Servo camServoX;
Servo camServoY;

#define directionServoPin 45
#define defaultMiddlePos 90
#define minDirServo 30
#define maxDirServo 150

int frontLastObstaclePos = 0; //-1 left | 0 unknown | 1right
int avoidDistance = 30; // in CM

long duration, distance, SideSensor_R, SideSensor_L, FrontSensor; //SonarSensor
String distanceLevel = "HIGH"; //HIGH - 30+cm | MID - 15cm-29cm | LOW - 5cm-14cm

float temperature;
float humidity;

void setup() {
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
  
  pinMode(trigPinSR, OUTPUT);
  pinMode(echoPinSR, INPUT);
  pinMode(trigPinSL, OUTPUT);
  pinMode(echoPinSL, INPUT);
  pinMode(trigPinFRONT,OUTPUT);
  pinMode(echoPinFRONT,INPUT);
  pinMode(43,INPUT);

  directionServo.attach(directionServoPin); 
  directionServo.write(defaultMiddlePos); 

  camServoX.attach(42);
  camServoX.write(90); //SET DEFAULT X POS
  camServoY.attach(43);
  camServoY.write(90); // SET DEFAULT Y POS
  
  Serial.begin(9600);
  delay(500);
}

void loop() {
remoteControl();
cameraPanTilt();
}

void setCameraPos(int x, int y){
  camServoX.write(x);
  camServoY.write(y);
}

void cameraPanTilt(){
  int panTiltPosX = pulseIn(10,HIGH); //PAN/TILT SERVO1
  int panTiltPosY = pulseIn(7,HIGH); //PAN/TILT SERVO2
  int minDirCamServo = 15;
  int maxDirCamServo = 165;

  int cameraX = map(panTiltPosX, 12, 977, minDirCamServo, maxDirCamServo);
  int cameraY = map(panTiltPosY, 12, 977, minDirCamServo, maxDirCamServo);

  setCameraPos(cameraX,cameraY);

  //----DEBUGGING ONLY----
  Serial.print("X: ");
  Serial.print(panTiltPosX);
  Serial.print("   Y: ");
  Serial.print(panTiltPosY);
  Serial.println();
}

void remoteControl(){
  int joyStickX = pulseIn(8,HIGH); //DIRECTION 
  int joyStickY = pulseIn(11,HIGH); //SPEED
  
  carSpeed = map(joyStickY, 12, 977, -255, 255);
  dirServoPos = map(joyStickX, 12, 977, minDirServo, maxDirServo);

  if(carSpeed > 255)
  carSpeed = 255;
  if(carSpeed<-255)
  carSpeed=-255;
  if(dirServoPos<0)
  dirServoPos = 0;
  if(dirServoPos>180)
  dirServoPos = 180;
  
  go(carSpeed);
  directionServo.write(dirServoPos);
  /*
  //----DEBUGGING ONLY----
  Serial.print("Speed: ");
  Serial.print(carSpeed);
  Serial.print("   Direction: ");
  Serial.print(dirServoPos);
  Serial.println();
  */
}


/*void readTempHum(){   -----Functia pentru citirea valorilor de la senzorul de TEMP/HUM

  int chk = DHT.read11(tempSensor);
  temperature = DHT.temperature;
  humidity = DHT.humidity;
}*/

void speedController(){ //Speed controller
 //TODO: NIVELE DE VITEZA (LIMITARI)
 //PARCAREA LATERALA SE VA REALIZA LA NIVELUL 1 DE VITEZA
}

void USSensors(){ //Folosind functia SonarSensor() + argumentele trigPin si echoPin, putem citi valorile fiecarui senzor separat
  SonarSensor(trigPinSR, echoPinSR);
  if(distance >= 100){
  SideSensor_R = 100;
  }
  SideSensor_R = distance;
  
  SonarSensor(trigPinSL, echoPinSL); 
  if(distance >= 100){
  SideSensor_L = 100;
  }
  SideSensor_L = distance;

  SonarSensor(trigPinFRONT, echoPinFRONT); 
  if(distance >= 100){
  FrontSensor = 100;
  }
  FrontSensor = distance;
}

int SonarSensor(int trigPin,int echoPin) //Functia este folosita pentru a citi valorile senzorilor de distanta
{
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH); //set timeout to 20000 *optional
if(duration != 0){
distance = (duration/2) / 29.1;
return distance;
}
else{
  distance = 100;
  return 100;
}
Serial.println(distance);
}

void go(int speedM) { // go(motorSpeed) motor driver control
  if (speedM > 0) {
    analogWrite(MOTOR1_PIN1, speedM);
    analogWrite(MOTOR1_PIN2, 0);
  } 
  else {
    analogWrite(MOTOR1_PIN1, 0);
    analogWrite(MOTOR1_PIN2, -speedM);
  }
}
