#include "LedControl.h"

LedControl lc = LedControl(9, 8, 7, 1);

/////////////////////////////////////////////////
byte LEFT_arry[8] = {
  B10000000,
  B11000000,
  B11100000,
  B11110000,
  B11111000,
  B11111100,
  B11111110,
  B11111111
};
//byte RIGHT_arry[8] = {
//  B00000001,
//  B00000010,
//  B00000100,
//  B00000000,
//  B00000000,
//  B00000000,
//  B00000000,
//  B00000000
//};
//byte FRONT_arry[8] = {
//  B00011000,
//  B01011010,
//  B10011001,
//  B00011000,
//  B00011000,
//  B00011000,
//  B00011000,
//  B00011000
//};
//byte TURN_arry[8] = {
//  B00000100,
//  B00001001,
//  B00010001,
//  B00100001,
//  B00100001,
//  B10101001,
//  B01110001,
//  B00100000
//};

void setup() {
  Serial.begin(9600);

//  pinMode(7, OUTPUT); // 도트 매트릭스 
//  pinMode(8, OUTPUT); // 도트 매트릭스 
//  pinMode(9, OUTPUT); // 도트 매트릭스 

  lc.shutdown(0, false);
  lc.setIntensity(0, 10);
  lc.clearDisplay(0);

}


void loop() {

//  digitalWrite(7, HIGH);
//  delay(300);
//  digitalWrite(8, HIGH);
//  delay(300);
//  digitalWrite(9, HIGH);
//  delay(300);
//  digitalWrite(7, LOW);
//  delay(300);
//  digitalWrite(8, LOW);
//  delay(300);
//  digitalWrite(9, LOW);
//  delay(300);
//
for (int row=0; row<8; row++){
  lc.setRow(0,row,LEFT_arry[row]);
  delay(10);  
}
//  display_LEFT(1);
//  delay(500);
//  display_LEFT(0);
//  delay(500);
//  display_RIGHT(1);
//  delay(500);
//  display_RIGHT(0);
//  delay(500);
//  display_FRONT(1);
//  delay(500);
//  display_FRONT(0);
//  delay(500);
//  display_TURN(1);
//  delay(500);
//  display_TURN(0);
//  delay(500);
}
//void display_LEFT(int a) {
//  if (a == 0) {
//    for (int i = 0; i < 8; i++) {
//      lc.setRow(0, i, LEFT_arry[i]);
//    }
//  }
//  else {
//    for (int i = 0; i < 8; i++) {
//      lc.setRow(0, i, B00000000);
//    }
//  }
//}


//void display_RIGHT(int a) {
//  if (a == 0) {
//    for (int i = 0; i < 8; i++) {
//      lc.setRow(0, i, RIGHT_arry[i]);
//    }
//  }
//  else {
//    for (int i = 0; i < 8; i++) {
//      lc.setRow(0, i, B00000000);
//    }
//  }
//}
//
//void display_FRONT(int a) {
//  if (a == 0) {
//    for (int i = 0; i < 8; i++) {
//      lc.setRow(0, i, FRONT_arry[i]);
//    }
//  }
//  else {
//    for (int i = 0; i < 8; i++) {
//      lc.setRow(0, i, B00000000);
//    }
//  }
//}
//
//void display_TURN(int a) {
//  if (a == 0) {
//    for (int i = 0; i < 8; i++) {
//      lc.setRow(0, i, TURN_arry[i]);
//    }
//  }
//  else {
//    for (int i = 0; i < 8; i++) {
//      lc.setRow(0, i, B00000000);
//    }
//  }
//}
