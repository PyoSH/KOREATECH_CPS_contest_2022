#include "VArduino.h"
#include <math.h>
#define TERMINATE '\r'
String cmd, data, buff;

int dv_mode = 0, av = 255;
bool x, y, z;
int r_d = 0, l_d = 0, f_d = 0;
const int b = 250;
const int a = 79;
int rs, ls, fs, prs, dir;
int rw, lw;
double prevTime = 0, chkTime = 0, dv_chkTime = 0, delayChkTime = 0;
double angle;
int d;

double target = 85;
double kp1 = 1, kp2 = 2;

int STEP = 0, pSTEP = 0;
int TURN = 0;
int m_step = 0;

int Limit(int a, int max, int min)
{
	return a > max ? max : a < min ? min : a;
}

void Linear(double ct, int ls, int v)
{
	if ((ct - prevTime) >= 0.1) {
		prevTime = ct;



		int ds = (rs - prs);
		dir = ds > 0 ? 1 : ds < 0 ? -1 : dir;
		prs = rs;

		int L = Limit((rs + ls + a), 32767, b);
		double rad = acos((double)b / L);
		angle = rad * RAD_TO_DEG * dir;
		d = (int)(rs * cos(rad));

		double err = kp1 * (target - d);
		double err2 = kp2 * (err - angle);


		rw = Limit((int)(v + err2), 255, 0);
		lw = Limit((int)(v - err2), 255, 0);





	}
}
bool SerialRead()
{
	while (Serial.available() > 0) {
		char ch = (char)Serial.read();
		if (ch == TERMINATE) {
			cmd = buff.substring(0, 1);
			if (buff.length() >= 2)
				data = buff.substring(1);
			else
				data = "0";
			buff = "";
			return true;
		}
		else {
			buff += ch;
		}
	}
	return false;
}
bool Forward(double ct)
{
	rw = lw = 150;
	if ((ct - chkTime) >= 0.1)
	{
		return true;
	}
	return false;
}

bool U_turn(double ct)
{
	/*
	rw = 0;
	lw = 0;
	rw = -180;
	lw = 180;
	if ((ct - chkTime) >= 0.7)
	{
		delayChkTime = ct;
		while ((ct - delayChkTime) <= 0.3)
		{
			ct = (double)millis() / 1000;

		}

		return true;
		m_step++;
	}
	return false;
	*/
	//
	switch (m_step)
	{
	case 0:

		rw = -180;
		lw = 180;
		if ((ct - chkTime) >= 0.6)
		{
			m_step++;
			chkTime = ct;
			delayChkTime = ct;
			while ((ct - delayChkTime) <= 0.3)
			{
				ct = (double)millis() / 1000;

			}
		}
		return false;

	case 1:
		if ((ct - prevTime) >= 0.1) {
			prevTime = ct;



			int ds = (rs - prs);
			dir = ds > 0 ? 1 : ds < 0 ? -1 : dir;
			prs = rs;

			int L = Limit((rs + ls + a), 32767, b);
			double rad = acos((double)b / L);
			angle = rad * RAD_TO_DEG * dir;
			d = (int)(rs * cos(rad));

			double err = kp1 * (target - d);
			double err2 = 1.5 * (err - angle);

			rw = Limit((int)(150 + err2), 255, 0);
			lw = Limit((int)(150 - err2), 255, 0);
		}

		if ((rs < 150) && (ls < 150))
		{
			m_step++;
			chkTime = ct;
		}
		return false;

	default:
		return true;

	}
}

void setup() {
	Serial.begin(9600);
	pinMode(12, OUTPUT);
	pinMode(13, OUTPUT);
	pinMode(2, INPUT);
	//analogWrite(10, 250);
	//analogWrite(11, 250);
	//Serial.print("start");
	//delay(1000);
	Serial.print("click button");

}

bool RL_Turn(double ct, int turn)
{
	switch (m_step)
	{
	case 0:
		if (turn == 1)
		{	
			rw = lw = 220; //150
			if ((ct - chkTime) >= 0.2) //0.4
			{
				m_step++;
				chkTime = ct;

			}
		}
		else if (turn == -1)
		{
			rw = lw = 150;

			if ((ct - chkTime) >= 0.1)
			{
				m_step++;
				chkTime = ct;
			}

		}

		return false;

	case 1:
		if (turn == 1)
		{
			rw = -220 * turn; //228
			lw = 220 * turn;
			if ((ct - chkTime) >= 0.65) //0.55
			{
				m_step++;
				chkTime = ct;
				/*
				delayChkTime = ct;
				while ((ct - delayChkTime) <= 0.3)
				{
					ct = (double)millis() / 1000;
				}
				*/
			}
		}
		
		else if (turn == -1)
		{
			rw = -228 * turn; //228
			lw = 228 * turn;
			if ((ct - chkTime) >= 0.55) //0.55
			{
				m_step++;
				chkTime = ct;
				/*
				delayChkTime = ct;
				while ((ct - delayChkTime) <= 0.3)
				{
					ct = (double)millis() / 1000;
				}
				*/
			}
		}
		
		return false;
	case 2:
		if (turn == 1)
		{
			rw = lw = 200; //153
			if ((ct - chkTime) >= 0.6) //1
			{
				m_step++;
				chkTime = ct;
			}
			
		}
		else if (turn == -1)
		{
			rw = lw = 200; //153
			if ((ct - chkTime) >= 0.6) //1
			{
				m_step++;
				chkTime = ct;
			}

		}
		return false;
		
	default:
		return true;
	}
}

bool ave_velo(double ct)
{
	av = 160;

	if ((ct - dv_chkTime) >= 1)
	{
		return true;


		//Serial.print("finish_decrease");
	}
	return false;
}

int count = 0;
int button = digitalRead(2);
int pre_button = 0;
int start = 0;

void loop() {

	int button = digitalRead(2);
	//Serial.print("click button");
	if (button == 1 && pre_button == 0 || start == 1)
	{
		start = 1;

		//시리얼로 수정하기
		bool isRecv = SerialRead();
		if (isRecv) {
			if (cmd == "t")
				target = (double)data.toInt();
			else if (cmd == "d")
				kp1 = (double)data.toInt() / 10.0;
			else if (cmd == "e")
				kp2 = (double)data.toInt() / 10.0;

		}

		double ct = (double)millis() / 1000;

		/*
		Serial.println(rs);
		Serial.println(ls);
		Serial.println(fs);
		Serial.println(rs_on);
		Serial.println(ls_on);
		Serial.println(fs_on);
		*/

		
		if (STEP != pSTEP)
		{

			char wBuff[64];
			int wLeng = sprintf(wBuff, "STEP:%d\r\n", STEP);
			Serial.write(wBuff, wLeng);

			Serial.println(x);
			Serial.println(y);
			Serial.println(z);
			Serial.println(r_d);
			Serial.println(l_d);
			Serial.println(f_d);


			dv_mode = 1;
			dv_chkTime = ct;

			pSTEP = STEP;
			/*
			if (STEP == 0)
			{
				chkTime = ct;
				while (1) {
					rs = analogRead(A0);
					ls = analogRead(A1);
					fs = analogRead(A2);
					ct = (double)millis() / 1000;


					int ds = (rs - prs);
					dir = ds > 0 ? 1 : ds < 0 ? -1 : dir;
					prs = rs;

					int L = Limit((rs + ls + a), 32767, b);
					double rad = acos((double)b / L);
					angle = rad * RAD_TO_DEG * dir;
					d = (int)(rs * cos(rad));

					double err = kp1 * (target - d);
					double err2 = kp2 * (err - angle);

					rw = Limit((int)(150 + err2), 255, 0);
					lw = Limit((int)(150 - err2), 255, 0);

					digitalWrite(12, rw < 0);
					digitalWrite(13, lw < 0);
					analogWrite(10, abs(rw));
					analogWrite(11, abs(lw));

					if ((ct - chkTime) >= 1.5)
						break;

				}

			}
			*/


		}


		rs = analogRead(A0);
		ls = analogRead(A1);
		fs = analogRead(A2);

		//좌우 전면 확인 플래그
		bool rs_on = (rs >= 170);//150  사고방지용
		bool ls_on = (ls >= 170);//150
		bool fs_on = (fs >= 90);


		//dv (감속모드 실행)
		if (dv_mode == 1 && STEP == 0)
		{
			//Serial.print(ct);
			if (ave_velo(ct))
			{
				dv_mode = 0;
				Serial.println(av);
				Serial.print("off");
				av = 250;
				if ((fs <= 450) && (fs >= 300))
					av = 200;
				else if (fs <= 300)
					av = 150;
			}

		}
		if (STEP == 0)
		{
			chkTime = ct;

			if (!rs_on && !ls_on && !fs_on) //막힘 + 막힘  + 막힘 
			{

				STEP = 1; //유턴 
			}

			else if (!rs_on && !ls_on && fs_on) //막 막 , 앞은 열
			{

				Linear(ct, ls, av); //전진
			}

			else if (!rs_on && ls_on && (fs <= 150)) //오 막 / 왼 열 / 앞 막 
			{

				STEP = 2; //왼쪽 가기
			}

			else if (!rs_on && ls_on && fs_on) //오 막 / 왼 열/ 앞 열
			{


				Linear(ct, b - rs - a, av);
				//STEP = 3; // 전진하기
			}
			else
			{
				x = rs_on;
				y = ls_on;
				z = fs_on;
				r_d = rs;
				l_d = ls;
				f_d = fs;

				STEP = 4; //전부 오른턴 
			}


		}
		else if (STEP == 1)
		{
			if (U_turn(ct))
				m_step = STEP = 0;
		}
		else if (STEP == 2)
		{
			if (RL_Turn(ct, -1))
				m_step = STEP = 0;
		}
		else if (STEP == 3)
		{
			STEP = 0;
		}
		else if (STEP == 4)
		{
			if (RL_Turn(ct, 1))
				m_step = STEP = 0;
		}


		digitalWrite(12, rw < 0);
		digitalWrite(13, lw < 0);
		analogWrite(10, abs(rw));
		analogWrite(11, abs(lw));
	}
	








}
