
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
//#define RPM_L 54
//#define RPM_R 57
#define SPEED 300
#define TICK_PER_CM 20.96//20//25//29.83
#define TICK_PER_DEG 4//.3
#define COUNT 50

DualVNH5019MotorShield md;

volatile unsigned int right_encoder_val = 0, left_encoder_val = 0;
int rEncStart = 0, lEncStart = 0;
float sensorInfo[6];
int blockDist[6];
int degree;

unsigned long nowTime = 0; //updated every loop()
unsigned long startTimeR = 0, startTimeL = 0; 
unsigned long timeTakenR = 0, timeTakenL = 0;
//double rpmR = 0, rpmL = 0;
double inputR = 0, outputR = 0, setpointR;
double inputL = 0, outputL = 0, setpointL;
//double kpR = 1, kiR = 0.001, kdR = 0;
double kpR = 1, /*1.056*/ kiR = 0, kdR = 0;
//value for RPMR = 74, RPML = 70
//double kpL = 1.05, kiL = 0.0022, kdL = 0.004;
double kpL = 1.07, kiL = 0, kdL = 0;
int deg; int dist;

String inputCmd;

PID myPIDR(&inputR, &outputR, &setpointR, kpR, kiR, kdR, DIRECT);
PID myPIDL(&inputL, &outputL, &setpointL, kpL, kiL, kdL, DIRECT);
ZSharpIR rightBack(RB, A1);
ZSharpIR leftBack(LB, A0);
ZSharpIR leftFront(LF, A3);
ZSharpIR frontMiddle(FM, A4);
ZSharpIR frontLeft(FL, A5);
ZSharpIR frontRight(FR, A2);


int count = 1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  md.init();
  pinMode(3, INPUT); //L
  pinMode(11, INPUT); //R
  PCintPort::attachInterrupt(R_encoder, RightEncoderInc, RISING);
  PCintPort::attachInterrupt(L_encoder, LeftEncoderInc, RISING);


  setpointR = 118;
  //setpointL = speedToRPML(SPEED);
  setpointL = 117.55; //114.9; //54.3;
  myPIDR.SetMode(AUTOMATIC);  
  myPIDL.SetMode(AUTOMATIC);
//  md.setSpeeds(SPEED,-SPEED); //L,R
  startTimeR = millis();
  startTimeL = millis();
}

void loop() {

  inputCmd = Serial.readStringUntil('@');
  while (!(inputCmd == NULL)){
    //Serial.println("loop");
    switch(inputCmd[0]){
      case 'W':
      {
        moveOne();
        delay(250);
        calibrationFB();
        delay(250);
        break;
      }
      case 'L':
      {
        turnLR();
        delay(250);
        break;
      }
      case 'R':
      {
        turnRR();
        delay(250);
        break;
      }
      case'F':
      {
        calibrationFB();
        delay(250);
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
        delay(250);
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
        delay(250);
        break;
      } 
      case 'U':
      {
        moveOne();
        break;
      }
      case 'H':
      {
        turnLR();
        break;
      }
      case 'K':
      {
        turnRR();
        break;
      }
    }
    inputCmd.remove(0,1);   
    }    
}
