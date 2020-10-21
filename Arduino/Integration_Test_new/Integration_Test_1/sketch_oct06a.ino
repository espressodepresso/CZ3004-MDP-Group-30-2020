//SENSORS
void sensorToRpi(){
  getSensorInfo(sensorInfo);
  for (int i=0;i<6;++i){   
    //L sensors return 0 blocks for dist <20
    if(i==3||i==4){
      //sensorInfo[i]=sensorInfo[i]-5;
      if((sensorInfo[i]>5)&&(sensorInfo[i]<13)){
        blockDist[i]=0;
      }
      else{
          blockDist[i] = sensorInfo[i]/10-1;
        }
    }

    if(i==5){
      sensorInfo[i] = sensorInfo[i]-5;
      if(sensorInfo[i]<22){
        blockDist[i]=0;
      }
      else{
        blockDist[i] = sensorInfo[i]/10;
        //blockDist[i]=5;
      }      
    }
    //tldr round to closest block
    //front sensors 
    if(i==0||i==1||i==2){
      if(sensorInfo[i]<12){
          blockDist[i] = 0;
        }
      else{
        sensorInfo[i]=sensorInfo[i];
        blockDist[i] = sensorInfo[i]/10;
      }
    }
    if (i==3||i==4){
      if (blockDist[i]>2){
        blockDist[i]=2;
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

void getSensorInfo(float sensorInfo[]){
  sensorInfo[0]=(frontLeft.distance());
  sensorInfo[1]=(frontMiddle.distance());
  sensorInfo[2]=(frontRight.distance());
  sensorInfo[3]=(leftFront.distance());
  sensorInfo[4]=(leftBack.distance());
  sensorInfo[5]=(rightBack.distance());
}



//MOVEMENT
void startPID(){
  //myPIDR.SetTunings(kpR,kiR,kdR);
  inputL = calcRPML();
  inputR = calcRPMR();
  myPIDL.Compute();
  myPIDR.Compute();
}

void moveForward(int dist){ //dist in 10 cm
  double target_ticks = 303 * dist; 

  right_encoder_val = left_encoder_val = 0;

  md.setSpeeds(SPEEDL, -SPEEDR);

  while(right_encoder_val < target_ticks){
    startPID();
    md.setM1Speed(rpmToSpeedL(inputL + outputL));
    md.setM2Speed(-rpmToSpeedR(inputR + outputR));
  }
  md.setBrakes(400,400);
  //delay(1000);
}

void moveOne(){
  double target_ticks = 290;//282; 

  right_encoder_val = 0;

  md.setSpeeds(SPEEDL, -SPEEDR);

  while(right_encoder_val < target_ticks){
    startPID();
    md.setM1Speed(rpmToSpeedL(inputL + outputL));
    md.setM2Speed(-rpmToSpeedR(inputR + outputR));
    //Serial.println(inputL+outputL);
  }
  md.setBrakes(400,400);
}

/*void moveBack(int dist){ //dist in cm
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
}*/

void turnRR(){ //90 R
  double target_ticks = 386;//386;//380;//404;//402; //385;//403; 

  right_encoder_val = left_encoder_val = 0;

  md.setSpeeds(-SPEEDL, -SPEEDR);

  while(right_encoder_val < target_ticks){
    startPID(); 
    md.setM1Speed(-rpmToSpeedL(inputL + outputL));
    md.setM2Speed(-rpmToSpeedR(inputR + outputR));
  }
  md.setBrakes(400,400);
  //delay(250);
}

void turnLR(){ //90 L
  double target_ticks = 392;//386;//409;//395;//405;//409;
  
  right_encoder_val = left_encoder_val = 0;

  md.setSpeeds(SPEEDL, SPEEDR);

  while(right_encoder_val < target_ticks){
    startPID(); 
    md.setM1Speed(rpmToSpeedL(inputL + outputL));
    md.setM2Speed(rpmToSpeedR(inputR + outputR));
  }
  md.setBrakes(400,400);
  //delay(250);
}

void turnLH(){
  double target_ticks = 787;//815;
  right_encoder_val = 0;
  md.setSpeeds(SPEEDL,SPEEDR);
  while(right_encoder_val < target_ticks){
    startPID();
    md.setM1Speed(rpmToSpeedL(inputL + outputL));
    md.setM2Speed(rpmToSpeedR(inputR + outputR));
  }
  md.setBrakes(400,400);
  //delay(250);
}

//CALIBRATION 
void calL(){
  double target_ticks = 1;
  right_encoder_val = left_encoder_val = 0;
  md.setSpeeds(SPEEDL,SPEEDR);
  while(right_encoder_val < target_ticks){}
  md.setBrakes(400,400);
}

void calR(){
  double target_ticks = 1;
  right_encoder_val = left_encoder_val = 0;
  md.setSpeeds(-SPEEDL,-SPEEDR);
  while(right_encoder_val < target_ticks){}
  md.setBrakes(400,400);
}

void calF(){
  double target_ticks = 1;
  right_encoder_val = 0;
  md.setSpeeds(SPEEDL,-SPEEDR);
  while(right_encoder_val < target_ticks){}
  md.setBrakes(400,400);
}

void calB(){
  double target_ticks = 1;
  right_encoder_val = 0;
  md.setSpeeds(-SPEEDL, SPEEDR);
  while(right_encoder_val < target_ticks){}
  md.setBrakes(400,400);
}

void startCal(){
  turnLH(); //turn 180
  delay(100);
  calibrationFBS(); //behind - 6.5
  delay(100);
  turnRR();
  delay(100);
  calibrationLRA();
  delay(100);
  calibrationFBS(); //behind - 6.5
  delay(100);
  turnRR(); //face front
  delay(100);
  calibrationLRA(); //calibrate left wall
  //delay(100);
}

void calibrationLRA(){
  //Serial.println("Calibrate");
  md.setSpeeds(0,0); //stop before calibrating
  getSensorInfo(sensorInfo);
  //actualDist();
  float fl = sensorInfo[3]; //front left
  float bl = sensorInfo[4]+1.5; //back left
  float diff = fl - bl; //neg = turn r, pos = turn l
  if (fl>20||bl>20){
    return;
  }
  //Serial.print("initial: "); Serial.println(diff);
  while(1){
    if(diff >= 0.3 && diff <= 9){ //positive error, turn L
      calL();
      getSensorInfo(sensorInfo);
      fl = sensorInfo[3]; 
      bl = sensorInfo[4]+1.5; 
      diff = fl - bl;
      //Serial.print("+ :"); Serial.println(diff);
    }
    else if(diff <= -0.3 && diff >= -9){ //negative error, turn R
      calR();
      getSensorInfo(sensorInfo);
      fl = sensorInfo[3]; 
      bl = sensorInfo[4]+1.5; 
      diff = fl - bl;
      //Serial.print("- :"); Serial.println(diff);
    }
    else{
      return;
    }
  }
}

void calibrationFBA(){
  md.setSpeeds(0,0); //stop before calibrating
  getSensorInfo(sensorInfo);
  //actualDist();
  float fl = sensorInfo[0]; //front left
  float fr = sensorInfo[2]; //front right
  float diff = fl - fr; 
  if (fl>20||fr>20){
    return;
  }
  //Serial.print("initial: "); Serial.println(diff);
  while(1){
    if(diff >= 0.4 && diff <= 8){ //positive error, turn L
      calR();
      getSensorInfo(sensorInfo);
      fl = sensorInfo[0]; 
      fr = sensorInfo[2]; 
      diff = fl - fr;
      //Serial.print("+ :"); Serial.println(diff);
    }
    else if(diff <= -0.4 && diff >= -8){ //negative error, turn R
      calL();
      getSensorInfo(sensorInfo);
      fl = sensorInfo[0]; 
      fr = sensorInfo[2]; 
      diff = fl - fr;
      //Serial.print("- :"); Serial.println(diff);
    }
    else{
      return;
    }
  } 
}

void calibrationLRD(){
  calibrationLRA();
  delay(250);
  turnLR();
  delay(250);
  calibrationFB();
  delay(250);
  turnRR();
  delay(250);
  calibrationLRA();
  //delay(250);
}

void calibrationFB(){
  md.setSpeeds(0,0); //stop before calibrating
  getSensorInfo(sensorInfo);
  if (sensorInfo[0]>22&&sensorInfo[1]>22||sensorInfo[1]>22&&sensorInfo[2]>22){
    return;
  }
  float closestSensor = min(sensorInfo[0],sensorInfo[1]);
  closestSensor = min(closestSensor, sensorInfo[2]);
  //Serial.println(closestSensor);
  while(1){
    if(closestSensor <= 6){ //too close, move back to 8cm mark
      calB();
      getSensorInfo(sensorInfo);
      closestSensor = min(sensorInfo[0],sensorInfo[1]);
      closestSensor = min(closestSensor, sensorInfo[2]);  
      //Serial.print("B :"); Serial.println(closestSensor);
    }
    else if(closestSensor <= 10 && closestSensor > 6.5){ //too far, move front to 8cm mark
      calF();
      getSensorInfo(sensorInfo);
      closestSensor = min(sensorInfo[0],sensorInfo[1]);
      closestSensor = min(closestSensor, sensorInfo[2]);  
      //Serial.print("F :"); Serial.println(closestSensor);
    }
    else if(closestSensor <= 16 && closestSensor>13.5){ //too close, move back to 8cm mark
      calB();
      getSensorInfo(sensorInfo);
      closestSensor = min(sensorInfo[0],sensorInfo[1]);
      closestSensor = min(closestSensor, sensorInfo[2]);  
      //Serial.print("B :"); Serial.println(closestSensor);
    }
    else if(closestSensor <= 21.5 && closestSensor > 16.5){ //too far, move front to 8cm mark
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
void calibrationFBF(){//for fastest path
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
    else if(closestSensor <= 10 && closestSensor > 7){ //too far, move front to 8cm mark
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

void calibrationFBS(){ //for the start
  md.setSpeeds(0,0); //stop before calibrating
  getSensorInfo(sensorInfo);
  float closestSensor = min(sensorInfo[0],sensorInfo[1]);
  closestSensor = min(closestSensor, sensorInfo[2]);
  //Serial.println(closestSensor);
  while(1){
    if(closestSensor <= 5.5){ //too close, move back to 8cm mark
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

// RPM STUFF
double rpmToSpeedR(double rpm_r){
  //double speedR = (rpm_r + 5.0856)/0.3123;
  //double speedR = (rpm_r + 6.6807)/0.3116;
  double speedR = (rpm_r + 9.5887)/0.433;
  return speedR;
}

/*double speedToRPMR(double speedR){
  //double rpmR = (0.3123 * speedR) - 5.0856;
  double rpmR = (0.3116 * speedR) - 6.6807;
  return rpmR;
}*/


double rpmToSpeedL(double rpm_l){
  //double speedL = (rpm_l + 7.7982)/0.3087;
  //double speedL = (rpm_l + 6.5911)/0.3113;
  double speedL = (rpm_l + 11.194)/0.4212;
  return speedL;
}

/*double speedToRPML(double speedL){
  //double rpmL = (0.3087 * speedL) - 7.7982;
  double rpmL = (0.3113 * speedL) - 6.5911;
  return rpmL;
}*/

double calcRPMR(){
  double duration = pulseIn(11,HIGH);
  //Serial.print("duration: "); Serial.println(duration);
  double rpmR = 36000000/562.25/duration;;
  //Serial.print("rpmR: "); Serial.println(rpmR);
  return rpmR;
}

double calcRPML(){
  double duration = pulseIn(3,HIGH);
  //Serial.print("duration: "); Serial.println(duration);
  double rpmL = 36000000/562.25/duration;;
  //Serial.print("rpmL: "); Serial.println(rpmL);
  return rpmL;
}


void RightEncoderInc(){
  right_encoder_val++;
}
  
void LeftEncoderInc(){
  left_encoder_val++;
}
