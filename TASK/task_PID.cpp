#include "VArduino.h"
// <0104> 문제 1: 상하 모터 제어 <해결!>
// -> 가능성 1 : 포트 설정 오류 ; 정답. 센서와 모터 핀이 바뀌어 설정되어있었다. 
//-------------------------------------------------------------------------------------
// <0105> 최소 기능의 모듈들 통합됨 -> 정지, 전진, 회전, 리프트 상승, 하강 
// 문제 )교차로에서 회전 시 45도를 돌아버린다. 
// -> 초기 define에서 STOP_TIME, TURN_TIME, WAIT_TIME을 조정해야 한다. 
// => STOP_TIME = 9000이 임계점인지 그 이전까지는 무조건 정해진 라인에서 멈춘다. 
// => TURN_TIME = 
// => WAIT_TIME = 

// <0109> 통합 & 시나리오 시퀀스
// 문제 1) 0105문제2
// -> 그냥 딜레이를 쓰던가 하는게?
// 
// 문제 2) 시퀀스 7, 21, 28에서 정지하고 적재해야 하는데 노빠꾸로 직진하면서 적재하는 현상 <<220번 줄>>, <해결!>
// -> NextMove 함수로 해결 시도 중...
//
// 문제 3) 라인 따라가는 기능 추가해야 함 <<111번 줄>>, <해결!>
// -> LineTrack 함수로 P제어 만들어 놓음
// -> 루프에 갇혀서 움직이지를 않는다.  => outputprocess()를 아래에 넣어주었다. 해결.
// -> 망할 루프만 벗어나게 해 주면 된다. => 해결. 


// 실험으로 결정할 파라미터들
#define SNS_MAX 800 // 센서 입력값 최대한계
#define SNS_MIN 25 // 센서 입력값 최소한계
#define MOT_MAX 254 // 모터 출력 최대한계
#define MOT_MIN 124 // 모터 출력 최소한계
#define STOP_TIME 2500 // 정지 시간
#define TURN_TIME 2500 // 회전 시간
#define WAIT_TIME 3000 // 움직임과 움직임 사이 대기 시간
#define THRES 600 // 검정 선 임계값 

#define TERMINATE '\r'

#define PUSH_BTN 2 // 푸쉬버튼

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
#define RANGE 100


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
int m_step =0; // 세부 동작 동기화를 위한 시퀀스
int m_pstep= -1; // 세부 동작_대조

int contest_move = 0; // 전체적인 움직임의 시퀀스 
int pcontest_move = 0; //  전체 시퀀스_대조

int count_line=0;
int move_target=0;
unsigned long chk_time; // 동작 조절 위한 시간

float error = 0;
float p_error = 0;
float acc_error = 0;
float P_GAIN = 1.25; // 0.305
float I_GAIN = 0.000; //0.0007
float D_GAIN = 0.0;
float dt = 0.0001;

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

	pinMode(PUSH_BTN, INPUT);
}

// 센서 입력 최신화
void InputProcess() {
	us = digitalRead(UP_SNS_PIN);
	ds = digitalRead(DN_SNS_PIN);
	rs = analogRead(R_LINE_SNS_PIN);
	ls = analogRead(L_LINE_SNS_PIN);
	//LineOn = ((rs >= THRES) && (ls >= THRES)); // 왼쪽 센서 & 오른쪽 센서가 임계값 이상이면 중간/정지선 
	//Rising = LineOn && !pLineOn; // 현재 신호와 이전 신호 비교 ~ 검정이 안잡히다가 잡히면 상승 ~
	//if (Rising)
	//	count_line++; 
	
	//pLineOn = LineOn; // 최신화
}


int Limit(int a, int max, int min) {
	return (a > max) ? max : ((a < min) ? min : a);
}

int Abs(int a) {
	return (a >= 0) ? a : (-1 * a);
}


void LineTrack(int &count_line) {
	//int error = (rs - ls)*254 / RANGE;
	//long e = 100 * s / (SNS_MAX - SNS_MIN);
	rs = analogRead(R_LINE_SNS_PIN);
	ls = analogRead(L_LINE_SNS_PIN);
	LineOn = ((rs >= THRES) && (ls >= THRES)); // 왼쪽 센서 & 오른쪽 센서가 임계값 이상이면 중간/정지선 
	Rising = LineOn && !pLineOn; // 현재 신호와 이전 신호 비교 ~ 검정이 안잡히다가 잡히면 상승 ~
	if (Rising){
		count_line++;
		}

	error = (ls - rs) * 254 / (SNS_MAX - SNS_MIN); 
	float P_value = P_GAIN * error ;
	float I_value = I_GAIN * acc_error*dt;
	float D_value = D_GAIN * (error - p_error) / dt;

	float control_value = P_value + I_value + D_value;

	//printf("!~ "); 에러 체크용
	
	//rw = ((MOT_MIN - MOT_MAX) * e / 100) + MOT_MAX; templet1
	//rw = ((MOT_MIN - MOT_MAX) * error / 100) + 170; Basic1
	//lw = ((MOT_MAX - MOT_MIN) * e / 100) + MOT_MAX; templet1
	//lw = ((MOT_MAX - MOT_MIN) * error / 100) + 170; Basic1
	
	rw = 200 + control_value;
	//rw = Limit(rw, MOT_MAX, MOT_MIN);
	lw = 200 - control_value;
	//lw = Limit(rw, MOT_MAX, MOT_MIN);
	
	pLineOn = LineOn; // 최신화
	acc_error += error;
	p_error = error;
	
}

//모터 출력 최신화
void OutputProcess() {
	digitalWrite(UP_MOT_PIN, (m_lift == 1) && (!us));
	digitalWrite(DN_MOT_PIN, (m_lift == -1) && (!ds));

	digitalWrite(R_DIR_PIN, rw < 0);
	digitalWrite(L_DIR_PIN, lw < 0);
	analogWrite(R_POW_PIN, rw);
	analogWrite(L_POW_PIN, lw);
}

bool NextMove(int&count_line, int count_tgt) {
	//count_line = 0; //이걸 여기에 놓는게 적절한지 봐야 함.
	static unsigned long chkT;
	bool stop_on = ((millis() - chkT) >= STOP_TIME);
	bool wait_on = ((millis() - chkT) >= WAIT_TIME);
	
	switch (m_step) {
		case 0: {
			while (m_step < 1) {
				LineTrack(count_line);
				OutputProcess();
				if (Rising) {
					m_step++;
					break;
					}
				}	
			return false;
		}		
	case 1: {
		while (0 < m_step&& m_step < 2) {
			LineTrack(count_line);
			if ((contest_move == 7 || contest_move == 19 || contest_move == 28))
				rw = lw = 25;
			OutputProcess();
			
			//printf(" ");
			if (stop_on || ((count_tgt -1) <count_line && count_line <= count_tgt)) {
				
				if ((contest_move == 7 || contest_move == 19 || contest_move == 21 || contest_move == 28) && count_line ==count_tgt) {
					rw = lw = 0;
					printf("show");
					m_step = 3;
					
				}	
				
				m_step++;
				break;
			}
		}
		//if (stop_on) m_step++;
		return false;
	}

	case 2: {
		while ( (1< m_step && m_step <= 2) ) {
			LineTrack(count_line);
			if ((count_tgt - 1) < count_line && count_line <= count_tgt) {
				//if (contest_move == 7 || contest_move == 19 || contest_move == 21 || contest_move == 28) {
				//	rw = lw = 0;
				//}
				rw = lw = 0;}
			//delay(350);
			OutputProcess();
			if ((rw==0) || ((count_tgt - 1) < count_line && count_line <= count_tgt)) {
				m_step++;
				break;
			}
		}
		rw = lw = 0;
		OutputProcess();
		return false;
	}

	default:
	{ 
		if (m_step == 3 ) {
			rw = lw = 0;
			OutputProcess();
			
		
		}
		
		printf("default");
		return true;
		}

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
		if (m_step >= 4) {
			rw = lw = 0;
		}
		return true;
	}
}

bool Turn_Point() {
	static unsigned long chkT;
	bool turn_on = ((millis() - chkT ) >= TURN_TIME);
	bool wait_on = ((millis() - chkT ) >= WAIT_TIME);
	switch (m_step) {
	case 0:
		rw = -100 ;
		lw = 100 ;
		if(ls < THRES) m_step++;
		if (rs < THRES) m_step++;
		return false;
	case 1:
		if (turn_on) m_step++;
		else m_step--;
		return false;
	case 2:
		rw = lw = 0;
		if (wait_on) m_step++;
		return false;
	default:
		if (m_step >= 3) {
			rw = lw = 0;
		}
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

void printdata(int step, int count) {
	//Serial.print("contest_move : "); Serial.print(contest_move);
	//Serial.print("count_line : "); Serial.print(count_line);
	printf("contest_move : "); printf("%d  ",contest_move);
	printf("m_step : "); printf("%d \n",m_step);
	printf("count_line : "); printf("%d    ", count_line); printf("target_cnt : "); printf("%d \n", move_target);
	printf("Right_motor : "); printf("%d   ", rw); printf("Left_motor : "); printf("%d \n\n", lw);

}


bool Move(int move_tgt) {
	move_target = move_tgt;
	if (NextMove(count_line, move_tgt) && (count_line > (move_tgt - 1) || count_line <= move_tgt)) {
		m_step = 0; count_line = 0;
		contest_move++;
		move_target = move_tgt = 0;
		return true;
	}

	else if (count_line > move_tgt || count_line < 0) {
		m_step = 0; count_line = move_tgt;
	    return false; }
	

	//else return false;
}

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

	if (contest_move != pcontest_move) {
		Serial.print("[STEP] : ");
		Serial.println(contest_move);
		pcontest_move = contest_move;
	}

	bool pb = digitalRead(PUSH_BTN);

	if (pb) {
		contest_move = 1;
	}

	switch (contest_move) {
		// -----------------------1st MISSION START, GO TO POINT "5"------------------------------
	case 1: {
		Move(1);
		printdata(contest_move, count_line);
	}
		  break;
	case 2: {// turn left
		if (Turn(-1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		  break;
	case 3: // 3칸 전진
	{Move(3); printdata(contest_move, count_line); }
	break;
	case 4: // 우회전
	{if (Turn(1)) {
		m_step = 0;
		contest_move++;
	}
	printdata(contest_move, count_line);
	}
	break;
	case 5: { // 4칸 전진
		Move(4); printdata(contest_move, count_line); }
		  break;
	case 6: { // 좌회전
		if (Turn(-1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		  break;
	case 7: {// 1칸 전진
		Move(1); printdata(contest_move, count_line); count_line = 0; }
		  break;
	case 8: {// lift up
		if (LiftUp()) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		  break;
		  // -----------------------1st PICK UP, GO TO POINT "C"------------------------------
	case 9: { // 좌회전
		if (Turn_Point()) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		  break;
	case 10: { // 5칸 전진
		Move(5); printdata(contest_move, count_line); }
		   break;
	case 11: { // 우회전
		if (Turn(1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	case 12: { //2칸 전진
		Move(2); printdata(contest_move, count_line); }
		   break;
	case 13: { // 좌회전
		if (Turn(-1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	case 14: { //1칸 직진
		Move(1); printdata(contest_move, count_line); }
		   break;
	case 15: { // 물건 하역
		if (LiftDown()) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	// -----------------------1st MISSION DONE, GO TO POINT "2"------------------------------
	case 16: { // 회전 to 돌아서 나가기
		if (Turn_Point()) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	case 17: { // 3칸 전진
		Move(3); printdata(contest_move, count_line); }
		   break;
	case 18: { // 좌회전
		if (Turn(-1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	case 19: { // 1칸 전진
		Move(1); printdata(contest_move, count_line); }
		   break;
	case 20: { // 우회전
		if (Turn(1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	case 21: { // 3칸 전진
		Move(3); printdata(contest_move, count_line); }
		   break;
	case 22: { // 물건 적재
		if (LiftUp()) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	// ----------------------2nd PICK UP,,  GO TO POINT "E"-----------------
	case 23: { // 회전 to 돌아서 나가기
		if (Turn_Point()) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	case 24: { // 5칸 전진
		Move(5); printdata(contest_move, count_line); }
		   break;
	case 25: { // 좌회전
		if (Turn(-1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	case 26: { //3칸 전진
		Move(3); printdata(contest_move, count_line); }
		   break;
	case 27: { //우회전
		if (Turn(1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	case 28: { // 1칸 전진
		Move(1); printdata(contest_move, count_line);
	}
		   break;
	case 29: { // 물건 하역
		if (LiftDown()) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	// -----------------------2nd MISSION DONE, GO TO GOAL------------------------------
	case 30: { // 회전 to 돌아서 나가기
		if (Turn_Point()) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	case 31: { // 4칸 전진
		Move(4);
		printdata(contest_move, count_line); }
		   break;
	case 32: { // 우회전
		if (Turn(1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	case 33: { // 직진 to 골인
		Move(1);
		printdata(contest_move, count_line); }
		   break;
		   // -----------------------GOAL IN---------------------------------------------------

	default: {
		contest_move = m_step = 0;
		acc_error = p_error = error = 0;
		rw = lw = 0;
		count_line = 0;
		move_target = 0;
		acc_error = p_error=error = 0;
		printdata(contest_move, count_line);
		OutputProcess(); // 출력 최신화
		if (pcontest_move == 33 && contest_move == 0) {
			exit(0);
		}
		break;
	}
	}

	OutputProcess(); // 출력 최신화

}
