#include <ZSharpIR.h>

#define sr1 A0
#define sr2 A2
#define sr3 A3
#define sr4 A5
#define lr1 A1
#define lr2 A4

#define DELAY_REFRESH     1000
ZSharpIR SharpIR(sr1, A0);
ZSharpIR SharpIR(sr2, A2);
ZSharpIR SharpIR(sr3, A3);
ZSharpIR SharpIR(sr4, A5);
ZSharpIR SharpIR(lr1, A1);
ZSharpIR SharpIR(lr2, A4);

void setup() {
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(SharpIR;
  delay(DELAY_REFRESH);
}
