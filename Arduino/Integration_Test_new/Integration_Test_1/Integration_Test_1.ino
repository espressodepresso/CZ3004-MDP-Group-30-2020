
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
double kpL = 1.063, kiL = 0, kdL = 0;
int deg; int dist;

String inputCmd;

PID myPIDR(&inputR, &outputR, &setpointR, kpR, kiR, kdR, DIRECT);
PID myPIDL(&inputL, &outputL, &setpointL, kpL, kiL, kdL, DIRECT);
ZSharpIR rightBack(RB, A1);
ZSharpIR leftBack(LB, A0);
ZSharpIR leftFront(LF, A3);
ZSharpIR frontMiddle(FM, A4);
ZSharpIR frontLeft(FL, A2);
ZSharpIR frontRight(FR, A5);

int count = 1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  md.init();
  pinMode(3, INPUT); //L
  pinMode(11, INPUT); //R
  PCintPort::attachInterrupt(R_encoder, RightEncoderInc, RISING);
  PCintPort::attachInterrupt(L_encoder, LeftEncoderInc, RISING);


  setpointR = 115.7;
  //setpointL = speedToRPML(SPEED);
  setpointL = 118; //114.9; //54.3;
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
        //moveForward(10);
        calibrationFB();
        break;
      }
      case 'L':
      {
        turnLR();
        break;
      }
      case 'R':
      {
        turnRR();
        break;
      }
      case'F':
      {
        calibrationFB();
        break;
      }
      case 'U':
      {
        calibrationLRA();
        break;
      }
      case 'C':
      {
        calibrationLRD();
        Serial.println("calibration done");
        break;
      }
      case 'K':
      {
        calibrationLRK();
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
        Serial.println("calibration done");
        break;
      }
      case 'e':
      {
        turnLH();
        break;
      } 
    }
    inputCmd.remove(0,1);   
    //delay(100);
    }    
}
/*
void sensorToRpi(){
  getSensorInfo(sensorInfo);
  for (int i=0;i<6;++i){
    
    //L sensors return 0 blocks for dist <20
    if(i==3||i==4){
      sensorInfo[i]=sensorInfo[i]-5;
      if((sensorInfo[i]>5)&&(sensorInfo[i]<12)){
        blockDist[i]=0;
      }
      else{
          blockDist[i] = sensorInfo[i]/10;
        }
    }


    if(i==5){
      sensorInfo[i]=sensorInfo[i]-5;
      blockDist[i] = sensorInfo[i]/10;
    }
    //tldr round to closest block

    if(i==0||i==1||i==2){
      if(sensorInfo[i]>12){
          blockDist[i] = sensorInfo[i]/10;
        }
      else{
        blockDist[i] = 0;
      }
    }

    //Long range sensor


    if(i!=5){
      if (blockDist[i]>3){
        blockDist[i]=3;
      }
    }
    else if(blockDist[i]>5){
      blockDist[i]=5;
    }
  }
  
  String toRpi = "[" + String(blockDist[0]);
  for (int j=1;j<6;++j){
    String sensorInfoString = String(blockDist[j]);
    toRpi = toRpi + "," + sensorInfoString;
  }
  toRpi =toRpi + "]";
  Serial.println(toRpi);
}

void actualDist(){
  getSensorInfo(sensorInfo);  
  String toRpi = "[" + String(sensorInfo[0]);
  for (int j=1;j<6;++j){
    String sensorInfoString = String(sensorInfo[j]);
    toRpi = toRpi + "," + sensorInfoString;
  }
  toRpi =toRpi + "]";
  Serial.println(toRpi);
}

void calibrationLR(){
  //Serial.println("Calibrate");
  md.setSpeeds(0,0); //stop before calibrating
  getSensorInfo(sensorInfo);
  float fl = sensorInfo[3]; //front left
  float bl = sensorInfo[4]; //back left
  float diff = fl - bl; //neg = turn r, pos = turn l
  //Serial.print("initial: "); Serial.println(diff);
  while(1){
    if(diff >= 1 && diff <= 8){ //positive error, turn L
      calL();
      getSensorInfo(sensorInfo);
      fl = sensorInfo[3]; 
      bl = sensorInfo[4]; 
      diff = fl - bl;
      //Serial.print("+ :"); Serial.println(diff);
    }
    else if(diff <= -1 && diff >= -8){ //negative error, turn R
      calR();
      getSensorInfo(sensorInfo);
      fl = sensorInfo[3]; 
      bl = sensorInfo[4]; 
      diff = fl - bl;
      //Serial.print("- :"); Serial.println(diff);
    }
    else{
      return;
    }
  }
}

void calibrationFB(){
  md.setSpeeds(0,0); //stop before calibrating
  getSensorInfo(sensorInfo);
  float closestSensor = min(sensorInfo[0],sensorInfo[1]);
  closestSensor = min(closestSensor, sensorInfo[2]);
  //Serial.println(closestSensor);
  while(1){
    if(closestSensor <= 7){ //too close, move back to 8cm mark
      calB();
      getSensorInfo(sensorInfo);
      closestSensor = min(sensorInfo[0],sensorInfo[1]);
      closestSensor = min(closestSensor, sensorInfo[2]);  
      //Serial.print("B :"); Serial.println(closestSensor);
    }
    else if(closestSensor <= 13 && closestSensor > 7.5){ //too far, move front to 8cm mark
      calF();
      getSensorInfo(sensorInfo);
      closestSensor = min(sensorInfo[0],sensorInfo[1]);
      closestSensor = min(closestSensor, sensorInfo[2]);  
      //Serial.print("F :"); Serial.println(closestSensor);
    }
    else{
      return;
    }
  }  
}

void calibrationFBS(){
  md.setSpeeds(0,0); //stop before calibrating
  getSensorInfo(sensorInfo);
  float closestSensor = min(sensorInfo[0],sensorInfo[1]);
  closestSensor = min(closestSensor, sensorInfo[2]);
  //Serial.println(closestSensor);
  while(1){
    if(closestSensor <= 6.5){ //too close, move back to 8cm mark
      calB();
      getSensorInfo(sensorInfo);
      closestSensor = min(sensorInfo[0],sensorInfo[1]);
      closestSensor = min(closestSensor, sensorInfo[2]);  
      //Serial.print("B :"); Serial.println(closestSensor);
    }
    else if(closestSensor <= 13 && closestSensor > 6.6){ //too far, move front to 8cm mark
      calF();
      getSensorInfo(sensorInfo);
      closestSensor = min(sensorInfo[0],sensorInfo[1]);
      closestSensor = min(closestSensor, sensorInfo[2]);  
      //Serial.print("F :"); Serial.println(closestSensor);
    }
    else{
      return;
    }
  }  
}



// SENSOR STUFF
void getSensorInfo(float sensorInfo[]){
  sensorInfo[0]=(frontLeft.distance());
  sensorInfo[1]=(frontMiddle.distance());
  sensorInfo[2]=(frontRight.distance());
  sensorInfo[3]=(leftFront.distance());
  sensorInfo[4]=(leftBack.distance());
  sensorInfo[5]=(rightBack.distance());
}


// RPM STUFF
double rpmToSpeedR(double rpm_r){
  //double speedR = (rpm_r + 5.0856)/0.3123;
  double speedR = (rpm_r + 6.6807)/0.3116;
  return speedR;
}

double speedToRPMR(double speedR){
  //double rpmR = (0.3123 * speedR) - 5.0856;
  double rpmR = (0.3116 * speedR) - 6.6807;
  return rpmR;
}


double rpmToSpeedL(double rpm_l){
  //double speedL = (rpm_l + 7.7982)/0.3087;
  double speedL = (rpm_l + 6.5911)/0.3113;
  return speedL;
}

double speedToRPML(double speedL){
  //double rpmL = (0.3087 * speedL) - 7.7982;
  double rpmL = (0.3113 * speedL) - 6.5911;
  return rpmL;
}

double calcRPMR(){
  rEncStart = right_encoder_val;
  while(right_encoder_val - rEncStart <= COUNT){}
  timeTakenR = millis() - startTimeR;
  startTimeR = millis();
  double rpmR = 2*(((COUNT/(double)timeTakenR) * 60000) / 562.25);
  return rpmR;
}

double calcRPML(){
  lEncStart = left_encoder_val;
  while(left_encoder_val - lEncStart <= COUNT){}
  timeTakenL = millis() - startTimeL;
  startTimeL = millis();
  double rpmL = 2*(((COUNT/(double)timeTakenL) * 60000) / 562.25);
  return rpmL;
}


void RightEncoderInc(){
  right_encoder_val++;
  }
  
void LeftEncoderInc(){
  left_encoder_val++;
  }



//MOVEMENT
void startPID(){
  inputL = calcRPML();
  inputR = calcRPMR();
  myPIDL.Compute();
  myPIDR.Compute();
}

void moveForward(int dist){
  double target_ticks = TICK_PER_CM * dist; 

  right_encoder_val = left_encoder_val = 0;

  md.setSpeeds(SPEED, -SPEED);

  while(right_encoder_val < target_ticks){
    startPID();
    md.setM1Speed(rpmToSpeedL(inputL + outputL));
    md.setM2Speed(-rpmToSpeedR(inputR + outputR));
  }
  md.setBrakes(400,400);
  //delay(1000);
}

void moveOne(){
  double target_ticks = 214.89; 

  right_encoder_val = 0;

  md.setSpeeds(SPEED, -SPEED);

  while(right_encoder_val < target_ticks){
    startPID();
    md.setM1Speed(rpmToSpeedL(inputL + outputL));
    md.setM2Speed(-rpmToSpeedR(inputR + outputR));
  }
  md.setBrakes(400,400);
  //delay(1000);
}

//dist in cm
void moveBack(int dist){
  double target_ticks = TICK_PER_CM * dist; 

  right_encoder_val = left_encoder_val = 0;

  md.setSpeeds(-SPEED, SPEED);

  while(right_encoder_val < target_ticks){
    startPID();  
    md.setM1Speed(-rpmToSpeedL(inputL + outputL));
    md.setM2Speed(rpmToSpeedR(inputR + outputR));
  }
  md.setBrakes(400,400);
  //delay(1000);
}

void turnL(int deg){
  double target_ticks = TICK_PER_DEG * deg;

  right_encoder_val = left_encoder_val = 0;

  md.setSpeeds(SPEED, SPEED);

  while(right_encoder_val < target_ticks){
    startPID(); 
    md.setM1Speed(rpmToSpeedL(inputL + outputL));
    md.setM2Speed(rpmToSpeedR(inputR + outputR));
  }
  md.setBrakes(400,400);
  //delay(1000);
}

void turnR(int deg){
  double target_ticks = TICK_PER_DEG * deg; 

  right_encoder_val = left_encoder_val = 0;

  md.setSpeeds(-SPEED, -SPEED);

  while(right_encoder_val < target_ticks){ 
    startPID();
    md.setM1Speed(-rpmToSpeedL(inputL + outputL));
    md.setM2Speed(-rpmToSpeedR(inputR + outputR));
  }
  md.setBrakes(400,400);
  //delay(1000);
}

void turnRR(){
  double target_ticks = 366; 

  right_encoder_val = left_encoder_val = 0;

  md.setSpeeds(-SPEED, -SPEED);

  while(right_encoder_val < target_ticks){
    startPID(); 
    md.setM1Speed(-rpmToSpeedL(inputL + outputL));
    md.setM2Speed(-rpmToSpeedR(inputR + outputR));
  }
  md.setBrakes(400,400);
}

void turnLR(){
  double target_ticks = 369;
  
  right_encoder_val = left_encoder_val = 0;

  md.setSpeeds(SPEED, SPEED);

  while(right_encoder_val < target_ticks){
    startPID(); 
    md.setM1Speed(rpmToSpeedL(inputL + outputL));
    md.setM2Speed(rpmToSpeedR(inputR + outputR));
  }
  md.setBrakes(400,400);
}

void turnLH(){
  double target_ticks = 750;
  right_encoder_val = 0;
  md.setSpeeds(SPEED,SPEED);
  while(right_encoder_val < target_ticks){
    startPID();
    md.setM1Speed(rpmToSpeedL(inputL + outputL));
    md.setM2Speed(rpmToSpeedR(inputR + outputR));
  }
  md.setBrakes(400,400);
}

// CALIBRATION STUFF
void calL(){
  double target_ticks = 1;
  right_encoder_val = left_encoder_val = 0;
  md.setSpeeds(SPEED,SPEED);
  while(right_encoder_val < target_ticks){}
  md.setBrakes(400,400);
}

void calR(){
  double target_ticks = 1;
  right_encoder_val = left_encoder_val = 0;
  md.setSpeeds(-SPEED,-SPEED);
  while(right_encoder_val < target_ticks){}
  md.setBrakes(400,400);
}

void calF(){
  double target_ticks = 1;
  right_encoder_val = 0;
  md.setSpeeds(SPEED,-SPEED);
  while(right_encoder_val < target_ticks){}
  md.setBrakes(400,400);
}

void calB(){
  double target_ticks = 1;
  right_encoder_val = 0;
  md.setSpeeds(-SPEED, SPEED);
  while(right_encoder_val < target_ticks){}
  md.setBrakes(400,400);
}

void startCal(){
  turnLH(); //turn 180
  calibrationFBS(); //behind - 6.5
  //sCalFB(); //calibrate behind
  turnRR();
  delay(20);
  calibrationLR();
  calibrationFBS(); //behind - 6.5
  //sCalFB(); //calibrate left wall
  turnRR(); //face front
  delay(20);
  calibrationLR(); //calibrate left wall
}*/
