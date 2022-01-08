#include "VArduino.h"
// <0104> ���� 1: ���� ���� ���� <�ذ�!>
// -> ���ɼ� 1 : ��Ʈ ���� ���� ; ����. ������ ���� ���� �ٲ�� �����Ǿ��־���. 
//-------------------------------------------------------------------------------------
// <0105> �ּ� ����� ���� ���յ� -> ����, ����, ȸ��, ����Ʈ ���, �ϰ� 
// ���� )�����ο��� ȸ�� �� 45���� ���ƹ�����. 
// -> �ʱ� define���� STOP_TIME, TURN_TIME, WAIT_TIME�� �����ؾ� �Ѵ�. 
// => STOP_TIME = 9000�� �Ӱ������� �� ���������� ������ ������ ���ο��� �����. 
// => TURN_TIME = 
// => WAIT_TIME = 

// <0109> ���� & �ó����� ������
// ���� 1) 0105����2
// -> �׳� �����̸� ������ �ϴ°�?
// 
// ���� 2) ������ 7, 21, 28���� �����ϰ� �����ؾ� �ϴµ� ����ٷ� �����ϸ鼭 �����ϴ� ���� <<220�� ��>>, <�ذ�!>
// -> NextMove �Լ��� �ذ� �õ� ��...
//
// ���� 3) ���� ���󰡴� ��� �߰��ؾ� �� <<111�� ��>>, <�ذ�!>
// -> LineTrack �Լ��� P���� ����� ����
// -> ������ ������ ���������� �ʴ´�.  => outputprocess()�� �Ʒ��� �־��־���. �ذ�.
// -> ���� ������ ����� �� �ָ� �ȴ�. => �ذ�. 


// �������� ������ �Ķ���͵�
#define SNS_MAX 800 // ���� �Է°� �ִ��Ѱ�
#define SNS_MIN 25 // ���� �Է°� �ּ��Ѱ�
#define MOT_MAX 254 // ���� ��� �ִ��Ѱ�
#define MOT_MIN 124 // ���� ��� �ּ��Ѱ�
#define STOP_TIME 6300 // ���� �ð�
#define TURN_TIME 5300 // ȸ�� �ð�
#define WAIT_TIME 7300 // �����Ӱ� ������ ���� ��� �ð�
#define THRES 600 // ���� �� �Ӱ谪 

#define TERMINATE '\r'

#define R_LINE_SNS_PIN A0 //������ ���� ���⼾��
#define L_LINE_SNS_PIN A1 //���� ���� ���⼾��

#define UP_MOT_PIN 7 //����Ʈ ��� ����
#define DN_MOT_PIN 8 //����Ʈ �ϰ� ����
#define UP_SNS_PIN 5 //����Ʈ ��¿Ϸ� Ȯ�� ����
#define DN_SNS_PIN 6 //����Ʈ �ϰ��Ϸ� Ȯ�� ����

#define R_POW_PIN 10 //������ ���� ��ũ 
#define L_POW_PIN 11 //���� ���� ��ũ 
#define R_DIR_PIN 12 //������ ���� ����
#define L_DIR_PIN 13 //���� ���� ����
#define RANGE 100


//����������
String cmd, data, buff; // �ø��� ��ſ뵵 ��������

int us; // ��¿Ϸ� ����
int ds; // �ϰ��Ϸ� ����
int rs; // ������ ���� ����
int ls; //  ���� ���� ����
int rw = 0; // �������� ���
int lw = 0; // �޹��� ���
int m_lift = 0; // 
bool LineOn; // ���� ������ Ȯ��
bool pLineOn; // ���� ������_����
bool Rising; // count �� �� ���� �� ���� ��� �ִ���
int m_step =0; // ���� ���� ����ȭ�� ���� ������
int m_pstep= -1; // ���� ����_����

int contest_move = 0; // ��ü���� �������� ������ 
int pcontest_move = 0; //  ��ü ������_����

int count_line=0;
int move_target=0;
unsigned long chk_time; // ���� ���� ���� �ð�

float P_GAIN = 0.3;

// ��� ����, �� �Ҵ�
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

// ���� �Է� �ֽ�ȭ
void InputProcess() {
	us = digitalRead(UP_SNS_PIN);
	ds = digitalRead(DN_SNS_PIN);
	rs = analogRead(R_LINE_SNS_PIN);
	ls = analogRead(L_LINE_SNS_PIN);
	//LineOn = ((rs >= THRES) && (ls >= THRES)); // ���� ���� & ������ ������ �Ӱ谪 �̻��̸� �߰�/������ 
	//Rising = LineOn && !pLineOn; // ���� ��ȣ�� ���� ��ȣ �� ~ ������ �������ٰ� ������ ��� ~
	//if (Rising)
	//	count_line++; 
	
	//pLineOn = LineOn; // �ֽ�ȭ
}


int Limit(int a, int max, int min) {
	return (a > max) ? max : ((a < min) ? min : a);
}

int Abs(int a) {
	return (a >= 0) ? a : (-1 * a);
}

// �� ���󰡱� <0109> ���� 3 -- P����� �������� �ߴµ� ���� �ʴ´�. ---����!!!!!!!!!!!!!
void LineTrack(int &count_line) {
	//int error = (rs - ls)*254 / RANGE;
	//long e = 100 * s / (SNS_MAX - SNS_MIN);
	rs = analogRead(R_LINE_SNS_PIN);
	ls = analogRead(L_LINE_SNS_PIN);
	LineOn = ((rs >= THRES) && (ls >= THRES)); // ���� ���� & ������ ������ �Ӱ谪 �̻��̸� �߰�/������ 
	Rising = LineOn && !pLineOn; // ���� ��ȣ�� ���� ��ȣ �� ~ ������ �������ٰ� ������ ��� ~
	if (Rising){
		count_line++;
		}

	float error = (ls - rs) * 254 / (SNS_MAX - SNS_MIN); 
	float control_value = P_GAIN * error;

	//printf("!~ "); ���� üũ��
	
	//rw = ((MOT_MIN - MOT_MAX) * e / 100) + MOT_MAX; templet1
	//rw = ((MOT_MIN - MOT_MAX) * error / 100) + 170; Basic1
	//lw = ((MOT_MAX - MOT_MIN) * e / 100) + MOT_MAX; templet1
	//lw = ((MOT_MAX - MOT_MIN) * error / 100) + 170; Basic1
	
	rw = 200 + control_value;
	//rw = Limit(rw, MOT_MAX, MOT_MIN);
	lw = 200 - control_value;
	//lw = Limit(rw, MOT_MAX, MOT_MIN);
	
	pLineOn = LineOn; // �ֽ�ȭ
	
}

//���� ��� �ֽ�ȭ
void OutputProcess() {
	digitalWrite(UP_MOT_PIN, (m_lift == 1) && (!us));
	digitalWrite(DN_MOT_PIN, (m_lift == -1) && (!ds));

	digitalWrite(R_DIR_PIN, rw < 0);
	digitalWrite(L_DIR_PIN, lw < 0);
	analogWrite(R_POW_PIN, rw);
	analogWrite(L_POW_PIN, lw);
}

/*
bool NextMove(int line_cnt, int count_tgt) {
	//count_line = 0; //�̰� ���⿡ ���°� �������� ���� ��.
	static unsigned long chkT;
	bool stop_on = ((millis() - chkT - 1000) >= STOP_TIME);
	bool wait_on = ((millis() - chkT - 1000) >= WAIT_TIME);
	switch (m_step) {
	case 0:
		LineTrack();
		if (Rising) m_step++;
		return false;
	case 1:
		//rw = lw = 100;
		LineTrack();
		if (stop_on) m_step++;
		return false;
	case 2:
		if ((count_tgt -1) <line_cnt && line_cnt <= count_tgt) {
			rw = lw = 0;
			if (contest_move == 7 || contest_move == 19 || contest_move == 21 || contest_move == 28) {
				rw = lw = 0;
			}
		}		
		if (wait_on) {
			
			m_step++;
		}
		return false;
	default:
	{
		//if (contest_move == 7 || contest_move == 19 || contest_move == 21 || contest_move == 28 && wait_on) {rw = lw = 0; }
		return true;
	}
		
	}	
} 


// �����ľ� ��.
bool LineMove(int line_cnt, int count_tgt) {
	static unsigned long chkT;
	bool stop_on = ((millis() - chkT - 1000) >= STOP_TIME);
	bool wait_on = ((millis() - chkT - 1000) >= WAIT_TIME);
	
	
	InputProcess();
	
	while (count_tgt > 0 && (0 <= line_cnt <= count_tgt)) {
		InputProcess();
		rw = lw = 100;
		printf("ew\n");
		
		if ((contest_move == 7 || contest_move == 19 || contest_move == 21 || contest_move == 28)
			|| (line_cnt == count_tgt) || (stop_on) || (stop_on && wait_on)) {
			rw = lw = 0;
			printf("wow0 \n");
			break;
		}
		else if (line_cnt < count_tgt) {
			LineTrack();
			printf("we\n");
		}
		return false;
	}

	return true;
}
*/

bool NextMove(int&count_line, int count_tgt) {
	//count_line = 0; //�̰� ���⿡ ���°� �������� ���� ��.
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
		while (0 < m_step&& m_step < 2 || !stop_on) {
			LineTrack(count_line);
			OutputProcess();
			//printf(" ");
			if (stop_on || ((count_tgt -1) <count_line && count_line <= count_tgt)) {
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
				if (contest_move == 7 || contest_move == 19 || contest_move == 21 || contest_move == 28) {
					rw = lw = 0;
				}
				rw = lw = 0;}
			delay(300);
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
		return true;
	}

	else if (count_line > move_tgt || count_line < 0) {
		m_step = 0; count_line = move_tgt;
	    return false; }
	

	//else return false;
}

/*/ ������ ����, �Ǵ� 
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

// �ø��� �Է� ����
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
	// �ø��� ��� ����
	bool isRecv = SerialRead();
	if (isRecv) {
		if (cmd == "s") // s0~ s5 �Է� �� �ش� ������ ������ ����
			//a_step = data.toInt();
			contest_move = data.toInt();
	}


	InputProcess(); // ���� ���� & ������ �ֽ�ȭ

	if (contest_move != pcontest_move) {
		Serial.print("[STEP] : ");
		Serial.println(contest_move);
		pcontest_move = contest_move;
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
	case 3: // 3ĭ ����
	{Move(3); printdata(contest_move, count_line); }
	break;
	case 4: // ��ȸ��
	{if (Turn(1)) {
		m_step = 0;
		contest_move++;
	}
	printdata(contest_move, count_line);
	}
	break;
	case 5: { // 4ĭ ����
		Move(4); printdata(contest_move, count_line); }
		  break;
	case 6: { // ��ȸ��
		if (Turn(-1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		  break;
	case 7: {// 1ĭ ����
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
	case 9: { // ��ȸ��
		if (Turn(-1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		  break;
	case 10: { // 5ĭ ����
		Move(5); printdata(contest_move, count_line); }
		   break;
	case 11: { // ��ȸ��
		if (Turn(1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	case 12: {
		Move(2); printdata(contest_move, count_line); }
		   break;
	case 13: {
		if (Turn(-1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	case 14: {
		Move(1); printdata(contest_move, count_line); }
		   break;
	case 15: {
		if (LiftDown()) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
		   // -----------------------1st MISSION DONE, GO TO POINT "2"------------------------------
	case 16: {
		if (Turn(-1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	case 17: {
		Move(3); printdata(contest_move, count_line); }
		   break;
	case 18: {
		if (Turn(-1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	case 19: {
		Move(1); printdata(contest_move, count_line); }
		   break;
	case 20: {
		if (Turn(1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	case 21: {
		Move(3); printdata(contest_move, count_line); }
		   break;
	case 22: {
		if (LiftUp()) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
		   // ----------------------2nd PICK UP,,  GO TO POINT "E"-----------------
	case 23: {
		if (Turn(1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	case 24: {
		Move(5); printdata(contest_move, count_line); }
		   break;
	case 25: {
		if (Turn(-1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	case 26: {
		Move(3); printdata(contest_move, count_line); }
		   break;
	case 27: {
		if (Turn(1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	case 28: {
		Move(1); printdata(contest_move, count_line);
	}
		   break;
	case 29: {
		if (LiftDown()) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
		   // -----------------------2nd MISSION DONE, GO TO GOAL------------------------------
	case 30: {
		if (Turn(-1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	case 31: {
		Move(4);
		printdata(contest_move, count_line); }
		   break;
	case 32: {
		if (Turn(1)) {
			m_step = 0;
			contest_move++;
		}
		printdata(contest_move, count_line); }
		   break;
	case 33: {
		Move(1);
		printdata(contest_move, count_line); }
		   break;
		   // -----------------------GOAL IN---------------------------------------------------

	default: {
		contest_move = m_step = 0;
		rw = lw = 0;
		count_line = 0;
		move_target = 0;
		printdata(contest_move, count_line);

		if (pcontest_move == 33 && contest_move == 0) {
			exit(0);
		}
		break;
	}
	}

	OutputProcess(); // ��� �ֽ�ȭ

}