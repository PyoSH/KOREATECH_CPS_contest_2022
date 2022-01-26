/***************
EEPROM을 초기화하거나 현재 들어있는 정보를 볼 수 있는 프로그램입니다.
초기화 하고 싶으면 14,15의 주석을 해제하고
확인만 하고 싶으면 14, 15의 주석을 처리하십시오.
****************/
#include <EEPROM.h>

void setup() {
  Serial.begin(9600);
  Serial.println("START");
  int index=50;//1024;
  for(int i=0; i<index;i++){
    Serial.print(i);Serial.print("  ");Serial.print(EEPROM.read(i)); Serial.print("   ");
    //EEPROM.write(i,255);
    //Serial.println((EEPROM.read(i)));
  }
}

void loop() {
 
}
