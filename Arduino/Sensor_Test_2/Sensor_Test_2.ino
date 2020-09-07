#include <ZSharpIR.h>

#define sr1 A0
#define sr2 A2
#define sr3 A3
#define sr4 A5
#define lr1 A1
#define lr2 A4

#define DELAY_REFRESH     1000

// ZSharpIR name (pin, model);

ZSharpIR long1(lr1, A1);
ZSharpIR long2(lr2, A4);
ZSharpIR short1(sr1, A0);
ZSharpIR short2(sr2, A2);
ZSharpIR short3(sr3, A3);
ZSharpIR short4(sr4, A5);
void setup() {
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(short1.analogOutput());
  Serial.println(short1.distance());
  Serial.println();
  delay(DELAY_REFRESH);
}
