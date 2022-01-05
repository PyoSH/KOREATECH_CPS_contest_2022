#include "VArduino.h"

#define lc A1
#define rc A0
#define lp 11
#define rp 10
#define ld 13
#define rd 12

float p = 0.645;
float i = 50;// 600000;// 4000;
float d = 0.000015;// 0.00000000015;

//float target = 25/412;
float error = 0.0;
float last_error = 0.0;
float acc_error = 0.0;
float dt = 0.00001;
float dftV = 200;

int min = -255; int max = 255;
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
	//analogWrite(lp, wl * colordiv / 100); analogWrite(rp, wr * colordiv / 100);
	analogWrite(lp, wl); analogWrite(rp, wr);
	//delay(t * 1000);
}
float Limits(float v) {
	if (v < min) return min;
	else if (v > max) return max;
	else return v;
}

float clc, crc;
int ercount = 0;

float cac() {
	clc = analogRead(lc), crc = analogRead(rc);
	error = (crc - clc) * 100 / range;
	float diff = error - last_error;
	last_error = error;
	acc_error += error;
	float control = p * error + i * acc_error * dt + d * diff / dt;

	if (clc != 25 && crc != 25 || lpwm != rpwm) {
		//printf("LEFT: %f, RIGHT: %f\n", clc, crc);
		//printf("error: %f, last_error: %f, acc_error: %f\n", error, last_error, acc_error);
		printf("control eql: %f + %f + %f\n", p * error, i * acc_error * dt, d * diff / dt);
		//printf("contorl : %f \n", control);
	}

	float wl = dftV + control, wr = dftV - control;
	wl = Limits(wl), wr = Limits(wr);
	lpwm = wl, rpwm = wr;

	if ((clc < 100) && (crc < 100) || (lpwm / rpwm < 1.35 && lpwm / rpwm>0.7)) { ercount++; }// Serial.print("strcount++  "); Serial.println(ercount);}
	else ercount = 0;
	if (ercount > 10) { tankLR(255, 255, dt); Serial.println("STRT"); }
	else tankLR(lpwm, rpwm, dt);
	return gtheta(wl, wr);


}



void setup() {
	Serial.begin(9600);

	pinMode(lp, OUTPUT); pinMode(rp, OUTPUT);
	pinMode(ld, OUTPUT); pinMode(rd, OUTPUT);
	printf("Main ver PID: %f, %f, %.10f\n\n", p, i, d);
	delay(2000);
}

float th1 = 0, th2 = 0, dth = 0;
int strcount = 0;
void loop() {
	th2 = cac();//later
	if (lpwm == 255 || rpwm == 255) { Serial.print("L: "); Serial.print(lpwm); Serial.print(",  R: "); Serial.println(rpwm); }
	if (clc != 25 && crc != 25 || lpwm != rpwm) { printf("L: %f,", lpwm); printf("  R: %f\n\n", rpwm); }

}