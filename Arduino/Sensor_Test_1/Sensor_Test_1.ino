#include <ZSharpIR.h>

#define ir A1
#define model 20150       //999 user type
#define DELAY_REFRESH     1000
ZSharpIR SharpIR(ir, model);

void setup() {
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(SharpIR.distance());
  delay(DELAY_REFRESH);
}
