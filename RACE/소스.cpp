#include "VArduino.h"

#define lc A1
#define rc A0
#define lp 11
#define rp 10
#define ld 13
#define rd 12

float p = 0.85;
float i = 0.08;
float d = 0.5;

//float target = 12;
float error = 0.0;
float last_error = 0.0;
float acc_error = 0.0;
float dt = 0.1;
float dftV = 90;

int min = -9999; int max = 9999;
float r = 0.035; float a = 0.136;

float lpwm, rpwm;

#define range 775
#define colordiv 255
/***************************************/

float gtheta(float wl, float wr) {
	return (wr - wl) * r / a;
}
void tankLR(float wl, float wr, float t = 0) {
	bool ldr = false, rdr = false;
	if (wl < 0) { ldr = true; wl *= -1; }
	if (wr < 0) { rdr = true; wr *= -1; }

	digitalWrite(ld, ldr); digitalWrite(rd, rdr);
	analogWrite(lp, wl*colordiv/100); analogWrite(rp, wr *colordiv/100);
	delay(t * 1000);
}
float Limits(float v) {
	if (v < min) return min;
	else if (v > max) return max;
	else return v;
}

float cac() {
	float clc = analogRead(lc), crc = analogRead(rc);
	error = (crc - clc)*100/range;
	float diff = error - last_error;
	last_error = error;
	acc_error += error;
	printf("LEFT: %f, RIGHT: %f", clc, crc);
	printf("error: %f, last_error: %f, acc_error: %f\n", error, last_error, acc_error);
		
	float control = p * error + i * acc_error * dt + d * diff / dt;
	printf("contorl : %f \n", control);
	float wl = dftV + control, wr = dftV - control;
	wl=Limits(wl),wr= Limits(wr);
	lpwm = wl, rpwm = wr;
	return gtheta(wl, wr);


}



void setup() {
	Serial.begin(9600);

	pinMode(lp, OUTPUT); pinMode(rp, OUTPUT);
	pinMode(ld, OUTPUT); pinMode(rd, OUTPUT);
	printf("PID: %f, %f, %,f\n\n", p, i, d);
	delay(2000);
}

float th = 0;
void loop() {
	th = cac();
	printf("L: %f,", lpwm); printf("  R: %f\n\n", rpwm);	tankLR(lpwm, rpwm, dt);
	
}