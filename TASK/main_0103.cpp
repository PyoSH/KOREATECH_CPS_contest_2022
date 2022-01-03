#include "VArduino.h"

#define TERMINATE '\t'

#define R_LINE_SNS_PIN A0
#define L_LINE_SNS_PIN A1

#define UP_MOT_PIN 5
#define DN_MOT_PIN 6
#define UP_SNS_PIN 7
#define DN_SNS_PIN 8

#define R_POW_PIN 10
#define L_POW_PIN 11
#define R_DIR_PIN 12
#define L_DIR_PIN 13

String cmd, data, buff;
bool LineOn(1);
int STEP = 0, pSTEP = 0;
int us, ds, rs, ls;
int speed;

bool SerialRead() {
	while (Serial.available() > 0) {
		char ch = (char)Serial.read();
		if (ch == TERMINATE) {
			cmd = buff.substring(0, 1);
			if (buff.length() >= 2)
				data = buff.substring(1);
			else
				data = "0";
			buff = "";
			return 1;
		}
		else{
			buff += ch;
		}
	}
	return false;
}

void InputProcess() {
	us = digitalRead(UP_SNS_PIN);
	ds = digitalRead(DN_SNS_PIN);
	rs = analogRead(R_LINE_SNS_PIN);
	ls = analogRead(L_LINE_SNS_PIN);
	LineOn = ((rs > 500) && (ls > 500));
}

void OutputProcess() {
	speed = (STEP == 2) ? 100 : 0;
	analogWrite(R_POW_PIN, speed);
	analogWrite(L_POW_PIN, speed);
	digitalWrite(UP_MOT_PIN, (STEP == 1));
	digitalWrite(DN_MOT_PIN, (STEP == 4));
}

void SequenceProcess() {
	switch (STEP) {
	case 0: // wait
		break;
	case 1: // lift up
		if (us == HIGH)
			STEP++;
		break;
	case 2: // forward
		if (LineOn == HIGH)
			STEP++;
		break;
	case 3: // stop
		if (speed == 0)
			STEP++;
		break;
	case 4: // lift down
		if (ds == HIGH)
			STEP++;
		break;
	case 5: // go to step 0
		STEP = 0;
		break;
	}
}

void setup() {
	Serial.begin(9600);
	printf("strt \n");
	pinMode(UP_SNS_PIN, INPUT);
	pinMode(DN_SNS_PIN, INPUT);
	pinMode(UP_MOT_PIN, OUTPUT);
	pinMode(DN_MOT_PIN, OUTPUT);
	pinMode(R_DIR_PIN, OUTPUT);
	pinMode(L_DIR_PIN, OUTPUT);
}

void loop() {
	printf("%f \n", Serial.read());

	/*bool isRecv = SerialRead();
	if (isRecv) {
		if (cmd == "s")
			STEP = data.toInt();
	}
	if (STEP != pSTEP) {
		char wBuff[128];
		int wLeng = sprintf(wBuff, "[STEP %d] \r\n", STEP);
		Serial.write(wBuff, wLeng);
		pSTEP = STEP;
	}
	InputProcess();
	SequenceProcess();
	OutputProcess();
	*/



	digitalWrite(R_POW_PIN, HIGH); // check for connection_test
	digitalWrite(L_POW_PIN, HIGH);
	delay(3000);
	digitalWrite(R_POW_PIN, LOW);
	digitalWrite(L_POW_PIN, LOW);
	delay(1000);
	printf("stopp \n");

} 
