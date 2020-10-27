
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

//change SPEED for start off straight
#define SPEEDL 323//323//319 (21, 6.31V) //310 //319  
#define SPEEDR 280//295 (21, 6.31V) //255
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
double inputR = 0, outputR = 0, setpointR;
double inputL = 0, outputL = 0, setpointL;
//double kpR = 1, kiR = 0.001, kdR = 0;
//double kpR = 1, /*1.056*/ kiR = 0, kdR = 0;
//value for RPMR = 74, RPML = 70
//double kpL = 1.05, kiL = 0.0022, kdL = 0.004;
//double kpL = 1.07, kiL = 0, kdL = 0;
//double kpL = 1.07, kiL = 0, kdL = 0.001;
double kpR = 0.49, kiR = 0, kdR = 0;
double kpL = 0.47, kiL = 0, kdL = 0;
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


  //setpointR = 113.64; //116;
  //setpointL = 121.137;//120;//119;//117.55; //114.9; //54.3;

  //change rpm for sudden change in speed during movement

  //setpoints BATT 20 6.22V
//  setpointR = 124;//118.4;//119; (20)
//  setpointL = 128;//128;
  
  //SETPOINTS batt 20 6.28V
  setpointR = 120;
  setpointL = 120;  

////  setpointS FOR BATT 21 6.31V
////  setpointR = 122;
////  setpointL = 128;

//  //setpointS FOR BATT 21 6.18V
//  setpointR = 130.5;//128.5;//122;
//  setpointL = 128;
  
  myPIDR.SetMode(AUTOMATIC);  
  myPIDL.SetMode(AUTOMATIC);
//  md.setSpeeds(SPEED,-SPEED); //L,R
  startTimeR = millis();
  startTimeL = millis();
}

void loop() {
  
//  moveForward(8);
//  delay(450);
//  turnRR();
//  delay(450);
//  turnRR();
//  delay(450);
//  moveForward(8);
//  delay(450);
//  turnLR();
//  delay(450);
//  turnLR();
//  delay(1000);  

//  turnRR();
//  delay(450);
//  turnRR();
//  delay(450)
//  turnRR();
//  delay(450);
//  turnRR();
//  delay(450);

 /* turnLR();
  delay(450);
  turnLR();
  delay(450)
  turnLR();
  delay(450);
  turnLR();
  delay(450);*/
  
  inputCmd = Serial.readStringUntil('@');
  while (!(inputCmd == NULL)){
    //Serial.println("loop");
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
        //moveOne();
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
