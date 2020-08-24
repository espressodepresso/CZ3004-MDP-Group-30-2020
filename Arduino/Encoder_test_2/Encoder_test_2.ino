#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"

#define R_encoder 3
#define L_encoder 11

DualVNH5019MotorShield md;

int right_encoder_val = 0, left_encoder_val = 0;

void RightEncoderInc(){right_encoder_val++;}
void LeftEncoderInc(){left_encoder_val++;}

//speed 100 left is 0.8375 of right
//speed 75 left is 0.777 of right

//work at speed 100 for now before doing pid


void setup() {
  Serial.begin(9600);
  md.init();
  PCintPort::attachInterrupt(R_encoder, RightEncoderInc, FALLING);
  PCintPort::attachInterrupt(L_encoder, LeftEncoderInc, FALLING);
}

void loop() {
  //M1 is left, M2 is right
  md.setM1Speed(100); 
  md.setM2Speed(-89);
  //order is 1,2
  //md.setSpeeds(100,-89);
  long ST = millis();
  getRPM(ST);

//  Serial.print("Left Enc: ");
//  Serial.println(left_encoder_val);
//
//  Serial.print("Right Enc: ");
//  Serial.println(right_encoder_val);
//
//  Serial.println();
//  delay(1000);
}

float getRPM(long ST){
  
  
  int startEnc = left_encoder_val;
  
  if (millis()-ST>1000){
    int curEnc = left_encoder_val;
    float rpm=(curEnc-startEnc)/2249*60;
    Serial.println(rpm);
  }
  delay(1000);
  
  
}
