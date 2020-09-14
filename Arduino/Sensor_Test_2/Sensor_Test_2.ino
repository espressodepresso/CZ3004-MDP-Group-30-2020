#include <ZSharpIR.h>
#define FR A0
#define LF A2
#define RF A3
#define LB A5
#define FC A1
#define RB A4

#define DELAY_REFRESH     1000

// ZSharpIR name (pin, model);

ZSharpIR frontCenter(FC, A1);
ZSharpIR rightBack(RB, A4);
ZSharpIR frontRight(FR, A0);
ZSharpIR leftFront(LF, A2);
ZSharpIR rightFront(RF, A3);
ZSharpIR leftBack(LB, A5);

int sensorInfo[6];
void setup() {
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  getSensorInfo(sensorInfo);
  Serial.print(sensorInfo[5]);
  Serial.print(' ');
  delay(DELAY_REFRESH);
}

void getSensorInfo(int sensorInfo[]){
  sensorInfo[0]=(frontCenter.distance());
  sensorInfo[1]=(frontRight.distance());
  sensorInfo[2]=(rightFront.distance());
  sensorInfo[3]=(rightBack.distance());
  sensorInfo[4]=(leftFront.distance());
  sensorInfo[5]=(leftBack.distance());
}
