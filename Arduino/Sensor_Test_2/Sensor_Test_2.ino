#include <ZSharpIR.h>
#define RB A1
#define LB A0
#define LF A3
#define FM A4
#define FL A2
#define FR A5 

#define DELAY_REFRESH 1000

// ZSharpIR name (pin, model);

ZSharpIR rightBack(RB, A1);
ZSharpIR leftBack(LB, A0);
ZSharpIR leftFront(LF, A3);
ZSharpIR frontMiddle(FM, A4);
ZSharpIR frontLeft(FL, A2);
ZSharpIR frontRight(FR, A5);


float sensorInfo[6];
int rawVoltage[6];
void setup() {
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  getSensorInfo(sensorInfo);
  getRawInfo(rawVoltage);
  for (int i=0;i<6;++i){
    Serial.print(sensorInfo[i]);
    Serial.print(' ');
  }
  Serial.println();
  for (int i=0;i<6;++i){
    Serial.print(rawVoltage[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(DELAY_REFRESH);
}

void getSensorInfo(float sensorInfo[]){
  sensorInfo[0]=(frontLeft.distance());
  sensorInfo[1]=(frontMiddle.distance());
  sensorInfo[2]=(frontRight.distance());
  sensorInfo[3]=(leftFront.distance());
  sensorInfo[4]=(leftBack.distance());
  sensorInfo[5]=(rightBack.distance());
}

void getRawInfo(int rawVoltage[]){
  rawVoltage[0]=(frontLeft.analogOutput());
  rawVoltage[1]=(frontMiddle.analogOutput());
  rawVoltage[2]=(frontRight.analogOutput());
  rawVoltage[3]=(leftFront.analogOutput());
  rawVoltage[4]=(leftBack.analogOutput());
  rawVoltage[5]=(rightBack.analogOutput());
}
