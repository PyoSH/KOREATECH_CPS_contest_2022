#include "VArduino.h"
// 0104 문제 1: 상하 모터 제어 <해결!>
// -> 가능성 1 : 포트 설정 오류 ; 정답. 센서와 모터 핀이 바뀌어 설정되어있었다. 
//-------------------------------------------------------------------------------------
// 0105 최소 기능의 모듈들 통합됨 -> 정지, 전진, 회전, 리프트 상승, 하강 
// 문제 1) 칸마다 정지의 기능이 첨엔 됐는데 안된다.  <해결!>
// -> 가능성 1 : 변수의 초기화가 제대로 안된다. 
// // 센서 간격 45로 조절. & 임계값 조절으로 해결. 
// 
// 문제 2) 교차로에서 회전 시 45도를 돌아버린다. 
// -> 초기 define에서 STOP_TIME, TURN_TIME, WAIT_TIME을 조정해야 한다. 
// => STOP_TIME = 9000이 임계점인지 그 이전까지는 무조건 정해진 라인에서 멈춘다. 
// => TURN_TIME = 
// => WAIT_TIME = 
// 
// 목표
// -> 정지선과 교차선의 차이를 인식할 수 있는가? 
// -> 정지선을 올바로 인식하고 멈출 수 있는가 -> 올바른 위치에서 리프트를 작동할 수 있는가
// -> 시퀀스를 실행했을 때 자동적으로 단계를 넘어갈 수 있는가
// 정지선에서 리프트를 사용하는 것을 달성하고, 목표 위치로 움직이는걸 처리해야 한다.

// 실험으로 결정할 파라미터들
#define SNS_MAX 800 // 센서 입력값 최대한계
#define SNS_MIN 25 // 센서 입력값 최소한계
#define MOT_MAX 254 // 모터 출력 최대한계
#define MOT_MIN 124 // 모터 출력 최소한계
#define STOP_TIME 1300 // 정지 시간
#define TURN_TIME 5300 // 회전 시간
#define WAIT_TIME 5300 // 움직임과 움직임 사이 대기 시간
#define THRES 600 // 검정 선 임계값 

#define TERMINATE '\r'

#define R_LINE_SNS_PIN A0 //오른쪽 라인 검출센서
#define L_LINE_SNS_PIN A1 //왼쪽 라인 검출센서

#define UP_MOT_PIN 7 //리프트 상승 모터
#define DN_MOT_PIN 8 //리프트 하강 모터
#define UP_SNS_PIN 5 //리프트 상승완료 확인 센서
#define DN_SNS_PIN 6 //리프트 하강완료 확인 센서

#define R_POW_PIN 10 //오른쪽 모터 토크 
#define L_POW_PIN 11 //왼쪽 모터 토크 
#define R_DIR_PIN 12 //오른쪽 모터 방향
#define L_DIR_PIN 13 //왼쪽 모터 방향

//전역변수들
String cmd, data, buff; // 시리얼 통신용도 변수선언

int us; // 상승완료 여부
int ds; // 하강완료 여부
int rs; // 오른쪽 검정 검출
int ls; //  왼쪽 검정 검출
int rw = 0; // 오른바퀴 출력
int lw = 0; // 왼바퀴 출력
int m_lift = 0; // 
bool LineOn; // 가로 검정선 확인
bool pLineOn; // 가로 검정선_대조
bool Rising; // count 할 때 검정 선 위에 계속 있는지
int m_step(0); // 세부 동작 동기화를 위한 시퀀스
int m_pstep(-1); // 세부 동작_대조
int a_step(0); // 전체적인 움직임의 시퀀스 
int a_pstep(-1); //  전체 시퀀스_대조

int contest_move(0);
int pcontest_move(0);

int count_line(0);
unsigned long chk_time; // 동작 조절 위한 시간

// 통신 시작, 핀 할당
void setup() {
	Serial.begin(9600);
	Serial.print("strt \n");
	pinMode(UP_SNS_PIN, INPUT);
	pinMode(DN_SNS_PIN, INPUT);
	pinMode(UP_MOT_PIN, OUTPUT);
	pinMode(DN_MOT_PIN, OUTPUT);
	pinMode(R_DIR_PIN, OUTPUT);
	pinMode(L_DIR_PIN, OUTPUT);
}

// 센서 입력 최신화
void InputProcess() {
	us = digitalRead(UP_SNS_PIN);
	ds = digitalRead(DN_SNS_PIN);
	rs = analogRead(R_LINE_SNS_PIN);
	ls = analogRead(L_LINE_SNS_PIN);
	LineOn = ((rs >= THRES) && (ls >= THRES)); // 왼쪽 센서 & 오른쪽 센서가 임계값 이상이면 중간/정지선 
	Rising = LineOn && !pLineOn; // 현재 신호와 이전 신호 비교 ~ 검정이 안잡히다가 잡히면 상승 ~
	if (Rising)
		count_line++; //이게 좋은 방법인가? 
	
	pLineOn = LineOn; // 최신화
}


int Limit(int a, int max, int min) {
	return (a > max) ? max : ((a < min) ? min : a);
}

int Abs(int a) {
	return (a >= 0) ? a : (-1 * a);
}

// 선 따라가기
void LineTrack() {
	int s = rs - ls;
	long e = 100L * s / (SNS_MAX - SNS_MIN);
	rw = (MOT_MIN - MOT_MAX) * e / 100 + MOT_MAX;
	rw = Limit(rw, MOT_MAX, MOT_MIN);
	lw = (MOT_MAX - MOT_MIN) * e / 100 + MOT_MAX;
	lw = Limit(rw, MOT_MAX, MOT_MIN);
}

//모터 출력 최신화
void OutputProcess() {
	digitalWrite(UP_MOT_PIN, (m_lift == 1) && (!us));
	digitalWrite(DN_MOT_PIN, (m_lift == -1) && (!ds));

	digitalWrite(R_DIR_PIN, rw < 0);
	digitalWrite(L_DIR_PIN, lw < 0);
	analogWrite(R_POW_PIN, Abs(rw));
	analogWrite(L_POW_PIN, Abs(lw));

	/*
	speed = (STEP == 2) ? 100 : 0;
	analogWrite(R_POW_PIN, speed);
	analogWrite(L_POW_PIN, speed);
	digitalWrite(UP_MOT_PIN, (STEP == 1));
	digitalWrite(DN_MOT_PIN, (STEP == 4)); */
}

bool NextMove() {
	static unsigned long chkT;
	bool stop_on = ((millis() - chkT - 1000) >= STOP_TIME);
	bool wait_on = ((millis() - chkT - 1000) >= WAIT_TIME);
	switch (m_step) {
	case 0:
		LineTrack();
		if (Rising) m_step++;
		return false;
	case 1:
		rw = lw = 100;
		LineTrack();
		if (stop_on) m_step++;
		return false;
	case 2:
		rw = lw = 0;
		if (wait_on) m_step++;
		return false;
	default:
		return true;

	}
}

// dir = right : 1/ left : -1 
bool Turn(int dir) {
	static unsigned long chkT;
	int right = 1, left = -1;
	bool turn_on = ((millis() - chkT - 500) >= TURN_TIME);
	bool wait_on = ((millis() - chkT - 500) >= WAIT_TIME);
	switch (m_step) {
	case 0:
		rw = -100 * dir;
		lw = 100 * dir;
		if ((dir == left) && (ls < THRES)) m_step++;
		else if ((dir == right) && (rs < THRES)) m_step++;
		return false;
	case 1:
		if ((dir == left) && (ls >= THRES)) m_step++;
		else if ((dir == right) && (rs >= THRES)) m_step++;
		return false;
	case 2:
		if (turn_on) m_step++;
		return false;
	case 3:
		rw = lw = 0;
		if (wait_on) m_step++;
		return false;
	default:
		return true;
	}
}

bool LiftUp() {
	static unsigned long chkT;

	bool wait_on = ((millis() - chkT) >= WAIT_TIME);
	switch (m_step) {
	case 0:
		m_lift = 1;
		if (us) m_step++;
		return false;
	case 1:
		m_lift = 0;
		if (wait_on) m_step++;
		return false;
	default:
		return true;
	}
}

bool LiftDown() {
	static unsigned long chkT;

	bool wait_on = ((millis() - chkT) >= WAIT_TIME);
	switch (m_step) {
	case 0: // 
		m_lift = -1;
		if (ds) m_step++;
		return false;
	case 1: //
		m_lift = 0;
		if (wait_on) m_step++;
		return false;
	default:
		return true;
	}
}

/*/ 시퀀스 정의, 판단 
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
}*/

// 시리얼 입력 관련
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
			return true;
		}
		else {
			buff += ch;
		}
	}
	return false;
}



void loop() {
	
    // 시리얼 통신 관련
	bool isRecv = SerialRead();
	if (isRecv) {
		if (cmd == "s") // s0~ s5 입력 시 해당 숫자의 시퀀스 실행
			//a_step = data.toInt();
			contest_move = data.toInt();
	}
	
	InputProcess(); // 센서 연결 & 데이터 최신화


	/*if (a_step != a_pstep) {
		Serial.print("[STEP] : ");
		Serial.println(a_step);
		a_pstep = a_step;


	switch (a_step) {
	case 1:
		if (NextMove())
			a_step = m_step = 0;
		break;
	case 2:
		if (Turn(1)) // turn right
			a_step = m_step = 0;
		break;
	case 3:
		if (Turn(-1)) // turn left
			a_step = m_step = 0;
		break;
	case 4:
		if (LiftUp())
			a_step = m_step = 0;
		break;
	case 5:
		if (LiftDown())
			a_step = m_step = 0;
		break;
	default:
		a_step = m_step = 0;
		rw = lw = 0;
		break;

	}
	} */
 
	if (contest_move != pcontest_move) {
		Serial.print("[STEP] : ");
		Serial.println(contest_move);
		pcontest_move = contest_move;
	}


	switch (contest_move) {
	// -----------------------1st MISSION START, GO TO POINT "5"------------------------------
	case 1: 
		if (NextMove() && count_line <= 1 && count_line > 0) {
			//contest_move = m_step = 0;
			m_step = 0;
			contest_move++;
		}
		//else if (NextMove() && count_line >= 3 && count_line < 4) {
		//	contest_move = m_step = 0;
		//}
		break;
	/*case 1:
		a_step = 1; 
		if (count_line <= 1 && count_line >0 )
			contest_move = m_step = 0;
		break;
		*/
	case 2: // turn left
		if (Turn(-1)) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 3: // 3칸 전진
		if (NextMove() && count_line >= 3 && count_line < 4) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 4: // 우회전
		if (Turn(1)) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 5: // 4칸 전진
		if (NextMove() && count_line >= 4 && count_line < 5) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 6: // 좌회전
		if (Turn(-1)) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 7: // 1칸 전진
		if (NextMove() && count_line > 0 && count_line <=1) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 8: // lift up
		if (LiftUp()) {
			m_step = 0;
			contest_move++;
		}
		break;
	// -----------------------1st PICK UP, GO TO POINT "C"------------------------------
	case 9: // 좌회전
		if (Turn(-1)) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 10: // 5칸 전진
		if (NextMove() && count_line >= 5 && count_line < 6) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 11: // 우회전
		if (Turn(1)) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 12:
		if (NextMove() && count_line >= 2 && count_line < 3) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 13:
		if (Turn(-1)) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 14:
		if (NextMove() && count_line > 0 && count_line <= 1) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 15:
		if (LiftDown()) {
			m_step = 0;
			contest_move++;
		}
		break;
	// -----------------------1st MISSION DONE, GO TO POINT "2"------------------------------
	case 16:
		if (Turn(-1)) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 17:
		if (NextMove() && count_line >= 3 && count_line < 4) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 18:
		if (Turn(-1)) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 19:
		if (NextMove() && count_line > 0 && count_line <= 1) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 20:
		if (Turn(1)) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 21:
		if (NextMove() && count_line >= 3 && count_line < 4) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 22:
		if (LiftUp()) {
			m_step = 0;
			contest_move++;
		}
		break;
	// ----------------------2nd PICK UP,,  GO TO POINT "E"-----------------
	case 23:
		if (Turn(1)) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 24:
		if (NextMove() && count_line >= 4 && count_line < 5) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 25:
		if (Turn(-1)) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 26:
		if (NextMove() && count_line >= 3 && count_line < 4) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 27:
		if (Turn(1)) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 28:
		if (NextMove() && count_line > 0 && count_line <= 1) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 29:
		if (LiftDown()) {
			m_step = 0;
			contest_move++;
		}
		break;
	// -----------------------2nd MISSION DONE, GO TO GOAL------------------------------
	case 30:
		if (Turn(-1)) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 31:
		if (NextMove() && count_line >= 4 && count_line < 5) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 32:
		if (Turn(1)) {
			m_step = 0;
			contest_move++;
		}
		break;
	case 33:
		if (NextMove())
			contest_move = m_step = 0;
		break; 
	// -----------------------GOAL IN---------------------------------------------------

	default:
		contest_move = m_step = 0;
		rw = lw = 0;
		break;
	}


	OutputProcess(); // 출력 최신화
		
} 
