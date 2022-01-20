#include "VArduino.h"
#include <math.h>
#define TERMINATE '\r'
String cmd, data, buff;

int dv_mode = 0;
bool x, y, z;
int r_d = 0, l_d = 0, f_d = 0;
int rs, ls, fs, prs, dir;
int rw, lw;
double prevTime = 0, chkTime = 0, dv_chkTime = 0, delayChkTime = 0;
double angle;
int d;

int STEP = 0, pSTEP = 0;
int TURN = 0;
int m_step = 0;
int count = 0;
int button = digitalRead(2);
int pre_button = 0;
int start = 0;


//////////////////튜닝 필수//////////////////////

//유턴, 좌우 턴 직진함수에 들어가 있는 턴 속도와 시간 튜닝 필수 

double target = 85; //벽과의 목표거리 (튜닝필요)
double kp1 = 1, kp2 = 2; //비례제어 (튜닝필요)
const int b = 250; //벽 사이간 거리 (튜닝필요)
const int a = 79;  //센서 간격 거리 (튜닝필요)
int av = 255; //평균 주행 속도 (원한다면 튜닝)

//벽 추종 직진 함수 (튜닝)
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

		//비례제어 파트 (kp1,kp2 튜닝 필)
		double err = kp1 * (target - d); // 벽 사이 거리 조정
		double err2 = kp2 * (err - angle);// 로봇의 기울기 각도 조정 


		rw = Limit((int)(v + err2), 255, 0);
		lw = Limit((int)(v - err2), 255, 0);





	}
}

//유턴함수(튜닝필)
bool U_turn(double ct)
{
	//case0 유턴 > case1 직진
	
	switch (m_step)
	{
	case 0:

		//rw,lw 속도값 (속도 튜닝 필)
		rw = -180;
		lw = 180;
		if ((ct - chkTime) >= 0.6) // 0.6은 회전시간 (시간 튜닝 필)
		{
			m_step++;
			chkTime = ct;
			delayChkTime = ct;
			while ((ct - delayChkTime) <= 0.3) //delay(300)과 동일 
			{
				ct = (double)millis() / 1000;

			}
		}
		return false;

	case 1://벽 추종 직진함수와 동일(속도 튜닝 필)
		if ((ct - prevTime) >= 0.1) {
			prevTime = ct;



			int ds = (rs - prs);
			dir = ds > 0 ? 1 : ds < 0 ? -1 : dir;
			prs = rs;

			int L = Limit((rs + ls + a), 32767, b);
			double rad = acos((double)b / L);
			angle = rad * RAD_TO_DEG * dir;
			d = (int)(rs * cos(rad));

			//kp1과 1.5를 적절한 비례값으로 변경
			double err = kp1 * (target - d);
			double err2 = 1.5 * (err - angle);

			//속도 150값을 적절한 값으로 변경 
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

//좌우 턴 함수(튜닝필)
bool RL_Turn(double ct, int turn)
{
	switch (m_step) //case0 직진 > case1 턴 > case2 직진
	{
	case 0:
		if (turn == 1) //우회전시 
		{
			rw = lw = 220; //튜닝 필
			if ((ct - chkTime) >= 0.2) //튜닝 필
			{
				m_step++;
				chkTime = ct;

			}
		}
		else if (turn == -1) //좌회전시 
		{
			rw = lw = 150; //튜닝 필

			if ((ct - chkTime) >= 0.1) //튜닝 필
			{
				m_step++;
				chkTime = ct;
			}

		}

		return false;

	case 1:
		if (turn == 1)
		{
			rw = -220 * turn; //좌우 튜닝필
			lw = 220 * turn;
			if ((ct - chkTime) >= 0.65) //시간 튜닝 필 
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
			rw = -228 * turn; //튜닝
			lw = 228 * turn;
			if ((ct - chkTime) >= 0.55) //튜닝
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
			rw = lw = 200; //튜닝
			if ((ct - chkTime) >= 0.6) //튜닝
			{
				m_step++;
				chkTime = ct;
			}

		}
		else if (turn == -1)
		{
			rw = lw = 200; //튜닝
			if ((ct - chkTime) >= 0.6) //튜닝
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

//상황 변경후 속도 감소 함수 (용도 : 속도가 빨라 컨트롤 안될때 사용)(튜닝 필) 
bool ave_velo(double ct)
{
	av = 160; //평균 속도 감소 (튜닝 필)

	if ((ct - dv_chkTime) >= 1) //시간 튜닝 필
	{
		return true;


		//Serial.print("finish_decrease");
	}
	return false;
}


/////////////////////////////////////////////////



//////////////노필요 구간/////////////

//리미트 함수
int Limit(int a, int max, int min)
{
	return a > max ? max : a < min ? min : a;
}
//시리얼로 제어계수 변경 (딱히 필요 없음)
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
//그냥 직진 함수 ( 사용안함)
bool Forward(double ct)
{
	rw = lw = 150;
	if ((ct - chkTime) >= 0.1)
	{
		return true;
	}
	return false;
}

///////////////////////////////////////


void setup() {
	Serial.begin(9600);
	pinMode(12, OUTPUT); //좌우센서 
	pinMode(13, OUTPUT);
	pinMode(2, INPUT); //스위치
	Serial.print("click button");

}


void loop() {

	int button = digitalRead(2);
	if (button == 1 && pre_button == 0 || start == 1)
	{
		start = 1; 

		//시리얼로 비례제어 값 변경 (필요 x)
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
		//디버깅용
		Serial.println(rs);
		Serial.println(ls);
		Serial.println(fs);
		Serial.println(rs_on);
		Serial.println(ls_on);
		Serial.println(fs_on);
		*/

		//상황 변경시 
		if (STEP != pSTEP)
		{

			char wBuff[64];
			int wLeng = sprintf(wBuff, "STEP:%d\r\n", STEP); //현재 상황 출력
			Serial.write(wBuff, wLeng);

			//디버깅용
			Serial.println(x);
			Serial.println(y);
			Serial.println(z);
			Serial.println(r_d);
			Serial.println(l_d);
			Serial.println(f_d);


			dv_mode = 1; //속도 저감모드 on
			dv_chkTime = ct;//속도 저감모드 시작 시간 저장 

			pSTEP = STEP;
			


		}


		rs = analogRead(A0);
		ls = analogRead(A1);
		fs = analogRead(A2);

		//////////////// 튜닝 필 /////////////////
		
		//좌,우,앞 센서 기준값 ex)170이하 거리를 벽으로 인식
		bool rs_on = (rs >= 170);
		bool ls_on = (ls >= 170);
		bool fs_on = (fs >= 90);

		//감속모드 실행
		if (dv_mode == 1 && STEP == 0)
		{
			//Serial.print(ct);
			if (ave_velo(ct))
			{
				dv_mode = 0; //감속모드 종료 
				Serial.println(av);
				Serial.print("off"); //종료 알림 
				av = 250; //평균속도 250으로 재설정 (필요시 튜닝)
				if ((fs <= 450) && (fs >= 300)) //센서값에 따른 평균 속도 설정 (필요시 튜닝)
					av = 200;
				else if (fs <= 300) //위와 동일 
					av = 150;
			}

		}

		////////////////////////////////////////
		

		//상황별 판단 
		if (STEP == 0)
		{
			chkTime = ct;

			if (!rs_on && !ls_on && !fs_on) //벽 + 벽 + 벽
			{

				STEP = 1; //유턴
			}

			else if (!rs_on && !ls_on && fs_on) //벽+ 벽 + 열 
			{

				Linear(ct, ls, av); //전진
			}

			else if (!rs_on && ls_on && (fs <= 150)) //벽+열+벽
			{

				STEP = 2; //왼쪽 턴 
			}

			else if (!rs_on && ls_on && fs_on) //벽+열+열
			{

				//전진
				Linear(ct, b - rs - a, av); //좌 센서 값은 측정 값이 아닌 계산값으로 대체 
				
			}
			else //그 외의 경우는 모두 우회전 
			{
				x = rs_on;
				y = ls_on;
				z = fs_on;
				r_d = rs;
				l_d = ls;
				f_d = fs;

				STEP = 4; //오른쪽 턴 
			}


		}
		else if (STEP == 1)//유턴
		{
			if (U_turn(ct))
				m_step = STEP = 0;
		}
		else if (STEP == 2)//좌회전
		{
			if (RL_Turn(ct, -1))
				m_step = STEP = 0;
		}
		else if (STEP == 4)//우회전
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