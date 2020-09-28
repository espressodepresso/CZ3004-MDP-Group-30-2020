#include <ZSharpIR.h>
#include <PID_v1.h>
#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#include <ArduinoQueue.h>

#define RB A1
#define LB A0
#define LF A3
#define FM A4
#define FL A2
#define FR A5 
#define R_encoder 11
#define L_encoder 3
#define RPM_L 70
#define RPM_R 74

DualVNH5019MotorShield md;

int right_encoder_val = 0, left_encoder_val = 0;
int rEncStart = 0, lEncStart = 0;
int sensorInfo[6];
int degree;

const unsigned int COUNT = 20;
unsigned long nowTime = 0; //updated every loop()
unsigned long startTimeR = 0, startTimeL = 0; 
double inputR, outputR, setpointR;
double inputL, outputL, setpointL;
double kpR = 1, kiR = 0, kdR = 0;
double kpL = 1.04, kiL = 0, kdL = 0.12;


struct cmd{
  char command;
  int arg;
};
String inputCmd ="";

PID myPIDR(&inputR, &outputR, &setpointR, kpR, kiR, kdR, DIRECT);
PID myPIDL(&inputL, &outputL, &setpointL, kpL, kiL, kdL, DIRECT);
ZSharpIR rightBack(RB, A1);
ZSharpIR leftBack(LB, A0);
ZSharpIR leftFront(LF, A3);
ZSharpIR frontMiddle(FM, A4);
ZSharpIR frontLeft(FL, A2);
ZSharpIR frontRight(FR, A5);
ArduinoQueue<cmd> commandQueue(5);

int count = 1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  md.init();
  startTimeR = millis();
  startTimeL = millis();
  PCintPort::attachInterrupt(R_encoder, RightEncoderInc, RISING);
  PCintPort::attachInterrupt(L_encoder, LeftEncoderInc, RISING);

  setpointR = RPM_R;
  setpointL = RPM_L;
  myPIDR.SetMode(AUTOMATIC);
  myPIDL.SetMode(AUTOMATIC);
}

void loop() {
  readCommands();
  struct cmd c;
  c = commandQueue.dequeue();
  delay(100);
    
  switch(inputCmd[0]){
    case 'W':
    {
      goForward();
      break;
    }
    case 'A':
    {
      turnLeft();
      break;
    }
    case 'D':
    {
      turnRight();
      break;
    }
    case 'L':
    {
      rotateLeft(degree);
      break;
    }
    case 'R':
    {
      rotateRight(degree);
      break;
    }
    case 'C':
    {
      calibration();
      break;
    }
    case 'V':
    {
      getSensorInfo(sensorInfo);
      for (int i=0;i<6;++i){
        Serial.print(sensorInfo[i]);
        Serial.print(' ');
      }
      Serial.print('|');
      break;
    }
  }
  delay(200);
}

void readCommands(){
  char newChar;
  inputCmd="";
  while (Serial.available()){
    newChar = Serial.read();
    inputCmd.concat(newChar);
    if (newChar == '\n'){
      break;
    }
  }
  Serial.print(inputCmd);
}



void goForward(){
  Serial.println("forward");
}
void turnLeft(){
  Serial.println("left");
}
void turnRight(){
  Serial.println("right");
}
void rotateLeft(int degree){
  Serial.print(degree);
  Serial.println("Rotate Left");
}
void rotateRight(int degree){
  Serial.print(degree);
  Serial.println("Rotate right");
}
void calibration(){
  Serial.println("Calibrate");
}


void getSensorInfo(int sensorInfo[]){
  sensorInfo[0]=(frontLeft.distance());
  sensorInfo[1]=(frontMiddle.distance());
  sensorInfo[2]=(frontRight.distance());
  sensorInfo[3]=(rightBack.distance());
  sensorInfo[4]=(leftFront.distance());
  sensorInfo[5]=(leftBack.distance());
}

double getRPM(long ST, char r){ //measured RPM
  double rpmR, rpmL, diviR, diviL;
  int startEncR, curEncR, startEncL, curEncL;
  
  startEncR = right_encoder_val;
  startEncL = left_encoder_val;
  Serial.print("startEncL");
  Serial.println(startEncL);
  //Serial.println(ST);
  while(1){
    if((millis()-ST)==1000){
    curEncR = right_encoder_val;
    curEncL = left_encoder_val;
    //Serial.print("curEnc");
    //Serial.println(curEnc);

    //rpmR
    diviR =(curEncR-startEncR)/562.25;
    rpmR = diviR*60;
    Serial.print("rpmR");
    Serial.println(rpmR);
    
    //rpmL
    diviL =(curEncL-startEncL)/562.25;
    rpmL = diviL*60;
    Serial.print("rpmL");
    Serial.println(rpmL);

    //return value
    if('r' == 'R') return rpmR;
    else return rpmL;
    }
  } 
  delay(1000);

}

double rpmToSpeedR(double rpm_r){
  double speedR = (rpm_r + 5.0856)/0.3123;
  return speedR;
}

double rpmToSpeedL(double rpm_l){
  double speedL = (rpm_l + 7.7982)/0.3087;
  return speedL;
}


void RightEncoderInc(){
  unsigned long timeTaken;
  right_encoder_val++;
  if (right_encoder_val - rEncStart == COUNT){
    timeTaken = startTimeR - nowTime;
    inputR = ((COUNT/timeTaken) * 1000 * 60) / 562.25;
    rEncStart += COUNT;
    startTimeR = nowTime;
    }
 }
  
void LeftEncoderInc(){
  unsigned long timeTaken;
  left_encoder_val++;
  if (left_encoder_val - lEncStart == COUNT){
    timeTaken = startTimeL - nowTime;
    inputL = ((COUNT/timeTaken) * 1000 * 60) / 562.25;
    lEncStart += COUNT;
    startTimeL = nowTime;
    }
}
