
#include <ZSharpIR.h>
#include <PID_v1.h>
#include <PinChangeInt.h>
#include <DualVNH5019MotorShield.h>

#define RB A1
#define LB A0
#define LF A3
#define FM A4
#define FL A2
#define FR A5 
#define R_encoder 11
#define L_encoder 3

//change SPEED for start off straight
#define SPEEDL 323 
#define SPEEDR 280

DualVNH5019MotorShield md;

volatile unsigned int right_encoder_val = 0, left_encoder_val = 0;
float sensorInfo[6];
int blockDist[6];

double inputR = 0, outputR = 0, setpointR;
double inputL = 0, outputL = 0, setpointL;
double kpR = 0.49, kiR = 0, kdR = 0;
double kpL = 0.47, kiL = 0, kdL = 0;
int dist;

String inputCmd;

PID myPIDR(&inputR, &outputR, &setpointR, kpR, kiR, kdR, DIRECT);
PID myPIDL(&inputL, &outputL, &setpointL, kpL, kiL, kdL, DIRECT);
ZSharpIR rightBack(RB, A1);
ZSharpIR leftBack(LB, A0);
ZSharpIR leftFront(LF, A3);
ZSharpIR frontMiddle(FM, A4);
ZSharpIR frontLeft(FL, A5);
ZSharpIR frontRight(FR, A2);

void setup() {
  Serial.begin(115200);
  md.init();
  pinMode(3, INPUT); //L
  pinMode(11, INPUT); //R
  PCintPort::attachInterrupt(R_encoder, RightEncoderInc, RISING);
  PCintPort::attachInterrupt(L_encoder, LeftEncoderInc, RISING);
  
  //SETPOINTS batt 20 6.28V
  setpointR = 77;
  setpointL = 75;  

  
  myPIDR.SetMode(AUTOMATIC);  
  myPIDL.SetMode(AUTOMATIC);
}

void loop() {
  
  inputCmd = Serial.readStringUntil('@');
  while (!(inputCmd == NULL)){
    switch(inputCmd[0]){
      case 'W':
      {
        moveOne();
        delay(450);
        calibrationFBA();
        calibrationFB();
        calibrationLRA();
        break;
      }
      case 'L':
      {
        turnLR();
        delay(450);
        calibrationFBA();
        calibrationFB();
        calibrationLRA();
        break;
      }
      case 'R':
      {
        turnRR();
        delay(450);
        calibrationFBA();
        calibrationFB();
        calibrationLRA();
        break;
      }
      case'F':
      {
        calibrationFBA();
        calibrationFB();
        delay(450);
        break;
      }
      case 'U':
      {
        calibrationLRA();
        delay(250);
        break;
      }
      case 'C':
      {
        calibrationLRD();
        delay(450);
        Serial.println("calibration done");
        break;
      }
      case 'V':
      {
        sensorToRpi();       
        break;
      }
      case 'X':
      {
        actualDist();
        break;
      }
      case 'S':
      {
        startCal();
        delay(250);
        Serial.println("calibration done");
        break;
      }
      case 'e':
      {
        turnLH();
        delay(350);
        break;
      } 
      case 'Y':
      {
        inputCmd.remove(0,1);
        int dist = (inputCmd[0]-'0');
        moveForward(dist);
        delay(200);
        calibrationFBF();
        delay(200);
        break;
      }
      case 'G':
      {
        turnLR();
        delay(200);
        calibrationLRA();
        delay(200);
        break;
      }
      case 'J':
      {
        turnRR();
        delay(200);
        calibrationLRA();
        delay(200);
        break;
      }
    }
    inputCmd.remove(0,1);   
    }   
}
