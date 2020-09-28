#include <ZSharpIR.h>
#include <PID_v1.h>
#include <PinChangeInt.h>
#include <DualVNH5019MotorShield.h>
#include <ArduinoQueue.h>

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
#define SPEED 200
#define TICK_PER_CM 29.83
#define TICK_PER_DEG 4.3
#define COUNT 50

DualVNH5019MotorShield md;

volatile unsigned int right_encoder_val = 0, left_encoder_val = 0;
int rEncStart = 0, lEncStart = 0;
int sensorInfo[6];
int degree;

unsigned long nowTime = 0; //updated every loop()
unsigned long startTimeR = 0, startTimeL = 0; 
unsigned long timeTakenR = 0, timeTakenL = 0;
//double rpmR = 0, rpmL = 0;
double inputR = 0, outputR = 0, setpointR;
double inputL = 0, outputL = 0, setpointL;
double kpR = 1, kiR = 0.001, kdR = 0;
//value for RPMR = 74, RPML = 70
//double kpL = 1.04, kiL = 0, kdL = 0.12;
//double kpL = 1.06, kiL = 0, kdL = 0.12;
double kpL = 1.62, kiL = 0.002, kdL = 0.004;
int deg; int dist;

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
  Serial.begin(115200);
  md.init();
  PCintPort::attachInterrupt(R_encoder, RightEncoderInc, RISING);
  PCintPort::attachInterrupt(L_encoder, LeftEncoderInc, RISING);


  setpointR = speedToRPMR(SPEED);
  setpointL = speedToRPML(SPEED);
  myPIDR.SetMode(AUTOMATIC);
  myPIDL.SetMode(AUTOMATIC);
//  md.setSpeeds(SPEED,-SPEED); //L,R
  startTimeR = millis();
  startTimeL = millis();
}

void loop() {
  readCommands();
  struct cmd c;
  c = commandQueue.dequeue();
  delay(100);
    
  switch(inputCmd[0]){
    case 'W':
    {
      moveForward(10);
      for (int i=0;i<6;++i){
        Serial.print(sensorInfo[i]);
        Serial.print(',');
      }
      Serial.println("");
      break;
    }
    case 'A':
    {
      turnL(90);
      getSensorInfo(sensorInfo);
      for (int i=0;i<6;++i){
        Serial.print(sensorInfo[i]);
        Serial.print(',');
      }
      Serial.println("");
      break;
    }
    case 'D':
    {
      turnR(90);
      getSensorInfo(sensorInfo);
      for (int i=0;i<6;++i){
        Serial.print(sensorInfo[i]);
        Serial.print(',');
      }
      Serial.println("");
      break;
    }
    case 'L':
    {
      break;
    }
    case 'R':
    {
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
        Serial.print(',');
      }
      Serial.println("");
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


void calibration(){
  Serial.println("Calibrate");
}


// SENSOR STUFF
void getSensorInfo(int sensorInfo[]){
  sensorInfo[0]=(frontLeft.distance());
  sensorInfo[1]=(frontMiddle.distance());
  sensorInfo[2]=(frontRight.distance());
  sensorInfo[3]=(leftFront.distance());
  sensorInfo[4]=(leftBack.distance());
  sensorInfo[5]=(rightBack.distance());
}


// RPM STUFF
double rpmToSpeedR(double rpm_r){
  double speedR = (rpm_r + 5.0856)/0.3123;
  return speedR;
}

double speedToRPMR(double speedR){
  double rpmR = (0.3123 * speedR) - 5.0856;
  return rpmR;
}


double rpmToSpeedL(double rpm_l){
  double speedL = (rpm_l + 7.7982)/0.3087;
  return speedL;
}

double speedToRPML(double speedL){
  double rpmL = (0.3087 * speedL) - 7.7982;
  return rpmL;
}

double calcRPMR(){
  rEncStart = right_encoder_val;
  while(right_encoder_val - rEncStart <= COUNT){}
  timeTakenR = millis() - startTimeR;
  startTimeR = millis();
  //Serial.print("timetakenR"); Serial.println(timeTakenR);
  double rpmR = 2*(((COUNT/(double)timeTakenR) * 60000) / 562.25);
  return rpmR;
}

double calcRPML(){
  lEncStart = left_encoder_val;
  while(left_encoder_val - lEncStart <= COUNT){}
  timeTakenL = millis() - startTimeL;
  startTimeL = millis();
  //Serial.print("timetakenL"); Serial.println(timeTakenL);
  double rpmL = 2*(((COUNT/(double)timeTakenL) * 60000) / 562.25);
  return rpmL;
}


void RightEncoderInc(){
  right_encoder_val++;
  /*if (right_encoder_val - rEncStart == COUNT){
    timeTakenR = nowTime - startTimeR;
    rEncStart += COUNT;
    startTimeR = nowTime;
    }*/
  }
  
void LeftEncoderInc(){
  left_encoder_val++;
  /*if (left_encoder_val - lEncStart == COUNT){
    timeTakenL = nowTime - startTimeL;
    lEncStart += COUNT;
    startTimeL = nowTime;
  }*/
}



//MOVEMENT
void moveForward(int dist){
  double target_ticks = TICK_PER_CM * dist; 

  right_encoder_val = left_encoder_val = 0;

  md.setSpeeds(SPEED, -SPEED);

  while(right_encoder_val < target_ticks){
    inputL = calcRPML();
    inputR = calcRPMR();
    //Serial.print("inputR: "); Serial.println(inputR);
    //Serial.print("inputL: "); Serial.println(inputL);
    //Serial.println(right_encoder_val);
    myPIDR.Compute();
    myPIDL.Compute();  
    //Serial.print("outputR: "); Serial.println(outputR);
    //Serial.print("outputL: "); Serial.println(outputL);
    md.setM2Speed(-rpmToSpeedR(inputR + outputR));
    md.setM1Speed(rpmToSpeedL(inputL + outputL));
  }
  md.setBrakes(400,400);
  delay(1000);
}

//dist in cm
void moveBack(int dist){
  double target_ticks = TICK_PER_CM * dist; 

  right_encoder_val = left_encoder_val = 0;

  md.setSpeeds(-SPEED, SPEED);

  while(right_encoder_val < target_ticks){
    inputL = calcRPML();
    inputR = calcRPMR();
    //Serial.print("inputR: "); Serial.println(inputR);
    //Serial.print("inputL: "); Serial.println(inputL);
    //Serial.println(right_encoder_val);
    myPIDR.Compute();
    myPIDL.Compute();  
    //Serial.print("outputR: "); Serial.println(outputR);
    //Serial.print("outputL: "); Serial.println(outputL);
    md.setM2Speed(rpmToSpeedR(inputR + outputR));
    md.setM1Speed(-rpmToSpeedL(inputL + outputL));
  }
  md.setBrakes(400,400);
  delay(1000);
}

void turnL(int deg){
  double target_ticks = TICK_PER_DEG * deg; 
  Serial.println(target_ticks);

  right_encoder_val = left_encoder_val = 0;

  md.setSpeeds(SPEED, SPEED);

  while(right_encoder_val < target_ticks){
    inputL = calcRPML();
    inputR = calcRPMR();
    //Serial.print("inputR: "); Serial.println(inputR);
    //Serial.print("inputL: "); Serial.println(inputL);
    //Serial.println(right_encoder_val);
    myPIDR.Compute();
    myPIDL.Compute();  
    //Serial.print("outputR: "); Serial.println(outputR);
    //Serial.print("outputL: "); Serial.println(outputL);
    md.setM2Speed(rpmToSpeedR(inputR + outputR));
    md.setM1Speed(rpmToSpeedL(inputL + outputL));
  }
  md.setBrakes(400,400);
  delay(1000);
}

void turnR(int deg){
  double target_ticks = TICK_PER_DEG * deg; 

  right_encoder_val = left_encoder_val = 0;

  md.setSpeeds(-SPEED, -SPEED);

  while(right_encoder_val < target_ticks){
    inputL = calcRPML();
    inputR = calcRPMR();
    //Serial.print("inputR: "); Serial.println(inputR);
    //Serial.print("inputL: "); Serial.println(inputL);
    //Serial.println(right_encoder_val);
    myPIDR.Compute();
    myPIDL.Compute();  
    //Serial.print("outputR: "); Serial.println(outputR);
    //Serial.print("outputL: "); Serial.println(outputL);
    md.setM2Speed(-rpmToSpeedR(inputR + outputR));
    md.setM1Speed(-rpmToSpeedL(inputL + outputL));
  }
  md.setBrakes(400,400);
  delay(1000);
}
