#include <math.h>
void setup() {
  Serial.begin(9600);
  delay(500);
  Serial.println("START");
}

const int d=250;
const int r=79;

double limit(double a){
  return a > 32767 ? 32767 : a <250 ? 250 : a;
}

void loop() {
  Serial.println("\n\n distances");
  double rs = analogRead(A0);
  double ls = analogRead(A1);
  double fs = analogRead(A2);
  double rad=acos((double)d/limit(rs+ls+r));
  Serial.print(ls);Serial.print(", ");
  Serial.print(rs);Serial.print(", ");
  Serial.println(fs);
  Serial.println(rad);
  double rd=rs*cos(rad);
  double ld=ls*cos(rad);
  Serial.print(rd); Serial.print(", ");
  Serial.println(ld);
  delay(1000);
}
