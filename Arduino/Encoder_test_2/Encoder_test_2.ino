#include <PID_v1.h>

#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"

#define R_encoder 11
#define L_encoder 3
#define RPM_L 70
#define RPM_R 74

DualVNH5019MotorShield md;

int right_encoder_val = 0, left_encoder_val = 0;
int rEncStart = 0, lEncStart = 0;
//int set_rpm = 70;

const unsigned int COUNT = 20;
unsigned long nowTime = 0; //updated every loop()
unsigned long startTimeR = 0, startTimeL = 0; 
double inputR, outputR, setpointR;
double inputL, outputL, setpointL;
double kpR = 1, kiR = 0, kdR = 0;
double kpL = 1.04, kiL = 0, kdL = 0.12;


PID myPIDR(&inputR, &outputR, &setpointR, kpR, kiR, kdR, DIRECT);
PID myPIDL(&inputL, &outputL, &setpointL, kpL, kiL, kdL, DIRECT);


//speed 100 left is 0.8375 of right
//speed 75 left is 0.777 of right

//work at 70 for now before doing pid


void setup() {
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
  
  //M1 is left, M2 is right
  md.setM2Speed(rpmToSpeedR(RPM_R));
  md.setM1Speed(-rpmToSpeedL(RPM_L));  //left one need reverse direction
}

void loop() {
  //double rpm, error, new_rpmL, new_rpmR;
  //long ST;
  //order is 1,2 (L,R)
  //md.setSpeeds(100,-89);
  //ST = millis(); //record the start time
  //Serial.print("ST");
  //Serial.println(ST);
  //rpm = getRPM(ST);
  //Serial.println(rpm);
  //error = set_rpm - rpm;
  //Serial.print("Error");
  //Serial.println(error);

  /*//RIGHT PID
  inputR = getRPM(ST,'R');
  myPIDR.Compute();
  Serial.print("outputR");
  Serial.println(outputR);
  new_rpmR = inputR + outputR;
  Serial.print("new rpmR");
  Serial.println(new_rpmR);
  md.setM2Speed(rpmToSpeedR(new_rpmR));

  //LEFT PID
  inputL = getRPM(ST,'L');
  myPIDL.Compute();
  Serial.print("outputL");
  Serial.println(outputL);
  new_rpmL = inputL + outputL;
  Serial.print("new rpmL");
  Serial.println(new_rpmL);
  md.setM1Speed(-rpmToSpeedL(new_rpmL));*/

  //inputR = getRPM(ST,'R');
  //inputL = getRPM(ST,'L');
  nowTime = millis();
  myPIDR.Compute();
  myPIDL.Compute();
  md.setM2Speed(rpmToSpeedR(outputR));
  md.setM1Speed(-rpmToSpeedL(outputL));
  //Serial.print("Rerror: "); Serial.println(outputR);
  //Serial.print("Lerror: "); Serial.println(outputL);

  
  //step_test();
  Serial.println();
  Serial.println();
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

/*void step_test(){
  Serial.println("enter step test");
  delay(5000);
  Serial.print("before rpm");
  Serial.println(getRPM(millis()));
  Serial.println("changing speed now");
  md.setM1Speed(250);
  while(1){
    Serial.print(millis());
    Serial.print("after rpm");
    Serial.println(getRPM(millis()));
    delay(100);
  }
}*/
