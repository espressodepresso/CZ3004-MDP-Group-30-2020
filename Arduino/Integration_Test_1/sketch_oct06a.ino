//SENSORS
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

void moveBack(int dist){ //dist in cm
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

void turnRR(){ //90 R
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

void turnLR(){ //90 L
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

//CALIBRATION 
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
}
