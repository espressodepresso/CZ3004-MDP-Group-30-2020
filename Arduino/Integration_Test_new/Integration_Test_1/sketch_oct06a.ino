//SENSORS
void sensorToRpi(){
  getSensorInfo(sensorInfo);
  for (int i=0;i<6;++i){   
    //L sensors return 0 blocks for dist <20
    if(i==3||i==4){
      if((sensorInfo[i]>5)&&(sensorInfo[i]<16)){
        blockDist[i]=0;
      }
      else{
        sensorInfo[i] = sensorInfo[i]-6;
        blockDist[i] = sensorInfo[i]/10;
      }
    }

    if(i==5){
      sensorInfo[i] = sensorInfo[i]-5;
      if(sensorInfo[i]<22){
        blockDist[i]=0;
      }
      else{
        blockDist[i] = sensorInfo[i]/10;
      }      
    }
    //tldr round to closest block
    //front sensors 
    if(i==0||i==1||i==2){
      if(sensorInfo[i]<15){
          blockDist[i] = 0;
        }
      else{
        sensorInfo[i]=sensorInfo[i]-5;
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
    else if(blockDist[i]>4){
      blockDist[i]=4;
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
}

void moveOne(){
  double target_ticks = 290;

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

void turnRR(){ //90 R
  double target_ticks = 393; 

  right_encoder_val = left_encoder_val = 0;

  md.setSpeeds(-SPEEDL, -SPEEDR);

  while(right_encoder_val < target_ticks){
    startPID(); 
    md.setM1Speed(-rpmToSpeedL(inputL + outputL));
    md.setM2Speed(-rpmToSpeedR(inputR + outputR));
  }
  md.setBrakes(400,400);
}

void turnLR(){ //90 L

  double target_ticks = 399;

  right_encoder_val = left_encoder_val = 0;

  md.setSpeeds(SPEEDL, SPEEDR);

  while(right_encoder_val < target_ticks){
    startPID(); 
    md.setM1Speed(rpmToSpeedL(inputL + outputL));
    md.setM2Speed(rpmToSpeedR(inputR + outputR));
  }
  md.setBrakes(400,400);
}

void turnLH(){
  double target_ticks = 795;//815;
  right_encoder_val = 0;
  md.setSpeeds(SPEEDL,SPEEDR);
  while(right_encoder_val < target_ticks){
    startPID();
    md.setM1Speed(rpmToSpeedL(inputL + outputL));
    md.setM2Speed(rpmToSpeedR(inputR + outputR));
  }
  md.setBrakes(400,400);
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
}

void calibrationLRA(){
  //Serial.println("Calibrate");
  md.setSpeeds(0,0); //stop before calibrating
  getSensorInfo(sensorInfo);
  //actualDist();
  float fl = sensorInfo[3]; //front left
  float bl = sensorInfo[4];//+0.6; //back left
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
      bl = sensorInfo[4];//+0.6; 
      diff = fl - bl;
      //Serial.print("+ :"); Serial.println(diff);
    }
    else if(diff <= -0.3 && diff >= -9){ //negative error, turn R
      calR();
      getSensorInfo(sensorInfo);
      fl = sensorInfo[3]; 
      bl = sensorInfo[4];//+0.6; 
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
  float fr = sensorInfo[2]+0.3; //front right
  float diff = fl - fr; 
  if (fl>18||fr>18){
    return;
  }
  //Serial.print("initial: "); Serial.println(diff);
  while(1){
    if(diff >= 0.4 && diff <= 6.5){ //positive error, turn L
      calR();
      getSensorInfo(sensorInfo);
      fl = sensorInfo[0]; 
      fr = sensorInfo[2]+0.3; 
      diff = fl - fr;
      //Serial.print("+ :"); Serial.println(diff);
    }
    else if(diff <= -0.4 && diff >= -6.5){ //negative error, turn R
      calL();
      getSensorInfo(sensorInfo);
      fl = sensorInfo[0]; 
      fr = sensorInfo[2]+0.3; 
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
}

void calibrationFB(){
  md.setSpeeds(0,0); //stop before calibrating
  getSensorInfo(sensorInfo);
  if (sensorInfo[0]>28&&sensorInfo[1]>28||sensorInfo[1]>28&&sensorInfo[2]>28){
    return;
  }
  float closestSensor = min(sensorInfo[0],sensorInfo[1]);
  closestSensor = min(closestSensor, sensorInfo[2]);
  //Serial.println(closestSensor);
  while(1){
    if(closestSensor <= 9){ //too close, move back to 8cm mark
      calB();
      getSensorInfo(sensorInfo);
      closestSensor = min(sensorInfo[0],sensorInfo[1]);
      closestSensor = min(closestSensor, sensorInfo[2]);  
      //Serial.print("B :"); Serial.println(closestSensor);
    }
    else if(closestSensor <= 12 && closestSensor > 9.5){ //too far, move front to 8cm mark
      calF();
      getSensorInfo(sensorInfo);
      closestSensor = min(sensorInfo[0],sensorInfo[1]);
      closestSensor = min(closestSensor, sensorInfo[2]);  
      //Serial.print("F :"); Serial.println(closestSensor);
    }
    else if(closestSensor <= 19 && closestSensor>15.5){ //too close, move back to 8cm mark
      calB();
      getSensorInfo(sensorInfo);
      closestSensor = min(sensorInfo[0],sensorInfo[1]);
      closestSensor = min(closestSensor, sensorInfo[2]);  
      //Serial.print("B :"); Serial.println(closestSensor);
    }
    else if(closestSensor <= 24 && closestSensor > 19.5){ //too far, move front to 8cm mark
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
    if(closestSensor <= 8.5){ //too close, move back to 8cm mark
      calB();
      getSensorInfo(sensorInfo);
      closestSensor = min(sensorInfo[0],sensorInfo[1]);
      closestSensor = min(closestSensor, sensorInfo[2]);  
      //Serial.print("B :"); Serial.println(closestSensor);
    }
    else if(closestSensor <= 12 && closestSensor > 9){ //too far, move front to 8cm mark
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
    if(closestSensor <= 8.5){ //too close, move back to 8cm mark
      calB();
      getSensorInfo(sensorInfo);
      closestSensor = min(sensorInfo[0],sensorInfo[1]);
      closestSensor = min(closestSensor, sensorInfo[2]);  
      //Serial.print("B :"); Serial.println(closestSensor);
    }
    else if(closestSensor <= 13 && closestSensor > 8.6){ //too far, move front to 8cm mark
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
  double speedR = (rpm_r + 6.6807)/0.3116;
  return speedR;
}

double rpmToSpeedL(double rpm_l){
  double speedL = (rpm_l + 6.5911)/0.3113;
  return speedL;
}

double calcRPMR(){
  double duration = pulseIn(11,HIGH);
  double rpmR = 36000000/562.25/duration;;
  return rpmR;
}

double calcRPML(){
  double duration = pulseIn(3,HIGH);
  double rpmL = 36000000/562.25/duration;;
  return rpmL;
}


void RightEncoderInc(){
  right_encoder_val++;
}
  
void LeftEncoderInc(){
  left_encoder_val++;
}
