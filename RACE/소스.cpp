#include "VArduino.h"
#include "math.h"

#define lc A1
#define rc A0
#define lp 11
#define rp 10
#define ld 13
#define rd 12
#define bl 6
#define br 5

float p = .24;
float i =1805;//100;// 600;// 600000;// 4000;
float d = 0.0000017;// 0.00000018;//0.0000001;// 0.0000015;// 0.00000000015;

//float target = 25/412;
float error = 0.0;
float last_error = 0.0;
float acc_error = 0.0;
float dt = 0.000001;
float dftV = 200;

int min = -255; int max = 255;
float r = 0.035; float a = 0.136;

float lpwm, rpwm;

#define range 775
#define colordiv 255
/***************************************/

float gtheta(float wl, float wr) {
	return (wr - wl);// *r / a;
}
void tankLR(float wl, float wr, float t = 0) {
	bool ldr = false, rdr = false;
	if (wl < 0) { ldr = true; wl *= -1; }
	if (wr < 0) { rdr = true; wr *= -1; }

	digitalWrite(ld, ldr); digitalWrite(rd, rdr);
	//analogWrite(lp, wl * colordiv / 100); analogWrite(rp, wr * colordiv / 100);
	analogWrite(lp, wl); analogWrite(rp, wr);
	delay(t * 1000);
}
bool limit;
float Limits(float v) {
	limit = false;
	if (v < min) return min;
	else if (v > max) return max;
	else limit = true; return v;
}

float clc, crc;
int ercount = 0;

float cac(float st=dt) {
	clc = analogRead(lc); crc = analogRead(rc);
	error = (crc - clc) * 255 / range;
	float diff = error - last_error;
	if(limit) acc_error += error;
	float control = p * error + i * acc_error * dt + d * diff / dt;

	if (clc != 25 && crc != 25 || lpwm != rpwm) {
		printf("LEFT: %f, RIGHT: %f\n", clc, crc);
		printf("error: %f, last_error: %f, acc_error: %f\n", error, last_error, acc_error);
		printf("control eql: %f + %f + %f\n", p * error, i * acc_error * dt, d * diff / dt);
		printf("contorl : %f \n", control);
	}

	float wl = dftV + control, wr = dftV - control;
	lpwm = Limits(wl); rpwm = Limits(wr); //Serial.print("BFR | L : ");  Serial.print(lpwm); Serial.print(" R: ");Serial.println(rpwm);
	lpwm += wr-rpwm; rpwm += wl-lpwm; //Serial.print("AFR | L : ");  Serial.print(lpwm); Serial.print(" R: "); Serial.println(rpwm);
	last_error = error;
	
	if ((clc < 80) && (crc < 80) ){//} || (lpwm / rpwm < 1.35 && lpwm / rpwm>0.7)) {
		ercount++;  //Serial.println("STRT");
}// Serial.print("strcount++  "); Serial.println(ercount);}
	else ercount = 0;
	if (ercount > 20) { tankLR(250, 250, st); acc_error = 0; }
	else tankLR(lpwm, rpwm, st);
	return gtheta(wl, wr);


}

/*float cacR(float st = dt) {
	clc = 825 / +825 % 2; crc = analogRead(rc);
	error = (crc - clc) * 255 / range;
	float diff = error - last_error;
	float control = p * error + i * acc_error * dt + d * diff / dt;

	if (clc != 25 && crc != 25 || lpwm != rpwm) {
		printf("LEFT: %f, RIGHT: %f\n", clc, crc);
		printf("error: %f, last_error: %f, acc_error: %f\n", error, last_error, acc_error);
		printf("control eql: %f + %f + %f\n", p * error, i * acc_error * dt, d * diff / dt);
		printf("contorl : %f \n", control);
	}

	float wl = dftV + control, wr = dftV - control;
	wl = Limits(wl); wr = Limits(wr);
	lpwm = wl; rpwm = wr;
	last_error = error;
	acc_error += error;
	if ((clc < 100) && (crc < 100) ){// || (lpwm / rpwm < 1.35 && lpwm / rpwm>0.7)) {
		ercount++;  //Serial.println("STRT");
	}// Serial.print("strcount++  "); Serial.println(ercount);}
	else ercount = 0;
	if (ercount > 10) { tankLR(190, 190, st); acc_error = 0; }
	else tankLR(lpwm, rpwm, st);
	return gtheta(wl, wr);
}
float cacL(float st = dt) {
	clc = analogRead(lc); crc = 825/2+825%2;
	error = (crc - clc) * 255.0 / range;
	float diff = error - last_error;
	float control = p * error + i * acc_error * dt + d * diff / dt;

	if (clc != 25 && crc != 25 || lpwm != rpwm) {
		printf("LEFT: %f, RIGHT: %f\n", clc, crc);
		printf("error: %f, last_error: %f, acc_error: %f\n", error, last_error, acc_error);
		printf("control eql: %f + %f + %f\n", p * error, i * acc_error * dt, d * diff / dt);
		printf("contorl : %f \n", control);
	}

	float wl = dftV + control, wr = dftV - control;
	wl = Limits(wl), wr = Limits(wr);
	lpwm = wl, rpwm = wr;
	last_error = error;
	acc_error += error;
	if ((clc < 100) && (crc < 100) || (lpwm / rpwm < 1.35 && lpwm / rpwm>0.7)) {
		ercount++;  //Serial.println("STRT");
	}// Serial.print("strcount++  "); Serial.println(ercount);}
	else ercount = 0;
	if (ercount > 10) { tankLR(190, 190, st); acc_error = 0; }
	else tankLR(lpwm, rpwm, st);
	return gtheta(wl, wr);
}*/

int checkDigital() {
	if (digitalRead(bl) && !digitalRead(br)) return 1;//left chek
	else if (digitalRead(br) && !digitalRead(bl)) return 2;//right check
	//else if (digitalRead(bl) && digitalRead(br))return 3;//straight check
	else return 0;//undetected
}


void setup() {
	Serial.begin(9600);

	pinMode(lp, OUTPUT); pinMode(rp, OUTPUT);
	pinMode(ld, OUTPUT); pinMode(rd, OUTPUT);
	pinMode(bl, INPUT); pinMode(br, INPUT);
	printf("Main ver PID: %f, %f, %.10f\n\n", p, i, d);
	delay(2000);
}

float th1 = 0, th2 = 0, dth = 0;
int sts = 0;
//float left_ths[10], right_ths[10];
double ths, left_ths, right_ths;

void loop() {

	/*if (sts != checkDigital()) {
		sts = checkDigital();
		tankLR(0, 0, 0.1);//감속 확장

		Serial.println("thresholdmod begin");//체크 이탈
		while (checkDigital()) { tankLR(lpwm * 0.8, rpwm * 0.8, dt); };

		Serial.println("Start measuring");//곡석 회전값 측정
		int sumcount = 0; bool breakpoint = false;
		for (int i = 0; i < 1000; i++) {
			ths += cac(0.001); sumcount++; if (checkDigital()) { breakpoint = true;  break; }
		}
		if (!breakpoint) {
			Serial.println("Ended the measure"); //tankLR(0, 0, 5);
			ths /= sumcount;// ths / 100 + ths - ths * (ths / 100); Serial.println(ths);
			if (ths < 0) ths *= -1;
			ths = round(ths * 100 / 100)*0.55;
			Serial.print("AVG ths: "); Serial.println(ths);//회전 평균계산


			if (sts == 1) { left_ths = 200.0 - ths; right_ths = 200.0; }
			else { right_ths = 200.0 - ths; left_ths = 200.0; }
			Serial.print("LEFTts: "); Serial.print(left_ths); Serial.print("   RIGHTts: "); Serial.println(right_ths);

			Serial.println("turning");
			while (!checkDigital()) { tankLR(left_ths, right_ths, dt); }
			//while (checkDigital()) { tankLR(lpwm * 0.8, rpwm * 0.8, dt); }
			}
		
		tankLR(0, 0, 0.1);
		for (int i = 0; i < 100; i++) { cac(0.001); }
		Serial.println("SEQUENCE END");
	}
	else*/ 
	if (checkDigital()) tankLR(0, 0, 0.3);
	th1 = cac(0.001);

}