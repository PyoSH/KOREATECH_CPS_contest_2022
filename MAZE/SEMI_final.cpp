// exp_주호
//#include"VArduino.h"
#include <math.h>
#include "LedControl.h"
#include <EEPROM.h>
#define TERMINATE '\r'
String cmd, data, buff;
LedControl lc = LedControl(9, 8, 7, 1);

int dv_mode = 0;
bool x, y, z;
int r_d = 0, l_d = 0, f_d = 0;
int rs, ls, fs, prs, dir;
int rw, lw;
double prevTime = 0, chkTime = 0, dv_chkTime = 0, delayChkTime = 0;
double step3_chktime = 0;
double angle;
int d;

int STEP = 0, pSTEP = 0;
int TURN = 0;
int m_step = 0;
int count = 0;
int button = digitalRead(2);
int pre_button = 0;
int start = 0;

bool nodecheck = false;
//char node[30];
char noderoute[30];
int nodeindex = 0;
int readindex = 0;

double target = 85;
double kp1 = 2, kp2 = 4;
const int b = 230;
const int a = 60;
int av = 255;


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


bool U_turn(double ct)
{
    //case0      > case1     
    //display_TURN(1);
    switch (m_step)
    {
    case 0:

        //rw,lw  ӵ    ( ӵ  Ʃ     )
        rw = -180;
        lw = 180;
        if ((ct - chkTime) >= 1.36) // 0.6   ȸ   ð  ( ð  Ʃ     ) 
          // (0) 0.6 (1) 0.8 (2) 1.5-230 (6) 1.46(   ͸          -250) (7) 1.4
        {
            m_step++;
            chkTime = ct;
            delayChkTime = ct;
            while ((ct - delayChkTime) <= 0.3) //delay(300)        
            {
                ct = (double)millis() / 1000;

            }
        }
        return false;

    case 1://             Լ        ( ӵ  Ʃ     )
        if ((ct - prevTime) >= 0.1) {
            prevTime = ct;

            int ds = (rs - prs);
            dir = ds > 0 ? 1 : ds < 0 ? -1 : dir;
            prs = rs;

            int L = Limit((rs + ls + a), 32767, b);
            double rad = acos((double)b / L);
            angle = rad * RAD_TO_DEG * dir;
            d = (int)(rs * cos(rad));

            //kp1   1.5            ʰ          
            double err = kp1 * (target - d);
            double err2 = 1.5 * (err - angle);

            // ӵ  150                        
            rw = Limit((int)(200 + err2), 255, 0); // (0)150 (1) 200
            lw = Limit((int)(200 - err2), 255, 0); // (0)150 (1) 200
        }

        if ((rs < 110) && (ls < 110))
        {
            m_step++;
            chkTime = ct;
        }
        return false;

    default: {
        //display_TURN(0);
        return true;
    }


    }
}

// ¿      Լ (Ʃ    )
bool RL_Turn(double ct, int turn)
{
    switch (m_step) //case0      > case1    > case2     
    {
    case 0:
        if (turn == 1) //  ȸ     
        {
            //display_RIGHT(1);
            rw = lw = 250; //Ʃ     
            if (((ct - chkTime) >= 1.2) || (fs <= 30)) //Ʃ      (0) 0.2 (1) 0.3 (2) 0.6 (3) 0.9 (4) 1.4( Batt_half -250) (5) 1.2(Batt_full) (6) 1.0
            {

                m_step++;
                chkTime = ct;

            }
        }
        else if (turn == -1) //  ȸ     
        {
            //display_LEFT(1);
            rw = lw = 250; //Ʃ      (0) 150 (1) 220

            if ((ct - chkTime) >= 1.3 || (fs <= 90)) //Ʃ      (0) 0.1 (1) 0.3 (2) 0.6 (3) 0.7 (4) 0.5(Batt_half -250) (5) 0.4 (6) 1.3(Batt_full)
            {
                /*if (fs <= 30) {
                  analogWrite(7, 100);
                }
                else {
                  analogWrite(7, 0);
                }*/
                m_step++;
                chkTime = ct;
            }

        }

        return false;

    case 1:
        if (turn == 1)
        {
            rw = -220 * turn; // ¿  Ʃ    
            lw = 220 * turn;
            if ((ct - chkTime) >= 0.8) // ð  Ʃ      (0) 0.65 (1) 0.85(   ͸     -250)
            {
                //display_RIGHT(0);
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
            rw = -235 * turn; //Ʃ   (1) -228
            lw = 235 * turn;
            if ((ct - chkTime) >= 0.8) //Ʃ   (0)0.55 (1) 0.85(   ͸     -250)
            {
                //display_LEFT(0);
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
            rw = lw = 240; //Ʃ  (0) 200
            if ((ct - chkTime) >= 1.5) //Ʃ   (0) 0.6 (1) 1.0
            {
                //display_RIGHT(0);
                m_step++;
                chkTime = ct;
            }

        }
        else if (turn == -1)
        {
            rw = lw = 240; //Ʃ  (0) 200
            if ((ct - chkTime) >= 1.5) //Ʃ   (0) 0.6 (1) 1.0
            {
                //display_LEFT(0);
                m_step++;
                chkTime = ct;
            }

        }
        return false;

    default:
        return true;
    }
}

//  Ȳ         ӵ        Լ  ( 뵵 :  ӵ           Ʈ    ȵɶ     )(Ʃ     ) 
bool ave_velo(double ct)
{
    av = 160; //     ӵ       (Ʃ     )

    if ((ct - dv_chkTime) >= 1) // ð  Ʃ     
    {
        return true;


        //Serial.print("finish_decrease");
    }
    return false;
}


/////////////////////////////////////////////////
byte bTemp;

const byte STRT_arry[] PROGMEM = {
  B10011001,
  B01100110,
  B01000010,
  B10011001,
  B10011001,
  B01000010,
  B01100110,
  B10011001
};

const byte LEFT_arry[] PROGMEM = {
  B00010000,
  B00110000,
  B01110000,
  B11111111,
  B11111111,
  B01110000,
  B00110000,
  B00010000
};
const byte RIGHT_arry[] PROGMEM = {
  B00001000,
  B00001100,
  B00001110,
  B11111111,
  B11111111,
  B00001110,
  B00001100,
  B00001000
};
const byte FRONT_arry[] PROGMEM = {
  B00011000,
  B00111100,
  B01111110,
  B11111111,
  B00011000,
  B00011000,
  B00011000,
  B00011000
};
const byte TURN_arry[] PROGMEM = {
  B00000000,
  B00110000,
  B01111100,
  B01000100,
  B01000100,
  B01011111,
  B01001110,
  B01000100
};

void DISPLAY_(int input) {
    if (input == 0) {
        //display_FRONT(1);
        for (int i = 0; i < 8; i++) {
            lc.setRow(0, i, B00000000);
        }
        for (int i = 0; i < 8; i++) {
            bTemp = pgm_read_byte_near(&(FRONT_arry[i]));
            //lc.setRow(0, i, FRONT_arry[i]);
            lc.setRow(0, i, bTemp);
        }
        //display_FRONT(0);
    }
    else if (input == 1) {
        //display_TURN(1);
        for (int i = 0; i < 8; i++) {
            lc.setRow(0, i, B00000000);
        }
        for (int i = 0; i < 8; i++) {
            bTemp = pgm_read_byte_near(&(TURN_arry[i]));
            //lc.setRow(0, i, TURN_arry[i]);
            lc.setRow(0, i, bTemp);
        }
        //display_TURN(0);
    }
    else if (input == 2) {
        //display_LEFT(1);
        for (int i = 0; i < 8; i++) {
            lc.setRow(0, i, B00000000);
        }
        for (int i = 0; i < 8; i++) {
            bTemp = pgm_read_byte_near(&(LEFT_arry[i]));
            //lc.setRow(0, i, LEFT_arry[i]);
            lc.setRow(0, i, bTemp);
        }
        //display_LEFT(0);
    }
    else if (input == 4) {
        //display_RIGHT(1);
        for (int i = 0; i < 8; i++) {
            lc.setRow(0, i, B00000000);
        }
        for (int i = 0; i < 8; i++) {
            bTemp = pgm_read_byte_near(&(RIGHT_arry[i]));
            //lc.setRow(0, i, RIGHT_arry[i]);
            lc.setRow(0, i, bTemp);
        }
        //display_RIGHT(0);
    }
    else if (input == 9) {
        //display_STRT(1);
        for (int i = 0; i < 8; i++) {
            lc.setRow(0, i, B00000000);
        }
        for (int i = 0; i < 8; i++) {
            bTemp = pgm_read_byte_near(&(STRT_arry[i]));
            lc.setRow(0, i, bTemp);
            //lc.setRow(0, i, STRT_arry[i])
        }
        //display_STRT(0);
    }


    /*
    else{
    lc.clearDisplay(0);} */
}
int Limit(int a, int max, int min)
{
    return a > max ? max : a < min ? min : a;
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
// ׳        Լ  (       )
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
    pinMode(12, OUTPUT); // ¿켾   
    pinMode(13, OUTPUT);
    pinMode(2, INPUT); //    ġ
    pinMode(7, OUTPUT); //   Ʈ   Ʈ     
    pinMode(8, OUTPUT); //   Ʈ   Ʈ     
    pinMode(9, OUTPUT); //   Ʈ   Ʈ     
    pinMode(4, OUTPUT);

    //   Ʈ   Ʈ     
    lc.shutdown(0, false);
    lc.setIntensity(0, 5);
    lc.clearDisplay(0); // ȭ    ʱ ȭ

    //mySerial.begin(9600); //          ø   
    Serial.print("click button");

}

void donode(int ndvl, int cndvl = 0) {
    if ((EEPROM.read(nodeindex) == 0 || EEPROM.read(nodeindex) == 255) || cndvl != 0) {
        int doublecheck = EEPROM.read(nodeindex);
        if (2 <= doublecheck && doublecheck <= 4) {
            switch (doublecheck) {
            case 4:
                if (cndvl == 4) EEPROM.write(nodeindex, 34);
                else if (cndvl == 3) {//잘못된길
                    EEPROM.write(nodeindex, 0);
                    nodeindex--;
                    donode(noderoute[nodeindex], noderoute[nodeindex]);
                }
                break;
                //case 3:
                //case3은 직좌뿐(4,3)을 위에서 했음.
            case 2:
                //무조건 자회전인데 아니다? 유턴하고 지우니깐 이전거가 아니라 넣으면 될 듯.
                EEPROM.write(noderoute[nodeindex], noderoute[nodeindex]);
            }
        }
        else EEPROM.write(nodeindex, cndvl);
    }
    noderoute[nodeindex] = ndvl;
    nodeindex++;
}


void loop() {
    //  lc.clearDisplay(0);
    int button = digitalRead(2);
    if (button == 1 && pre_button == 0 || start == 1)
    {
        start = 1;

        // ø                     ( ʿ  x)
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


        //Serial.println(rs);
        //Serial.println(ls);
        //Serial.println(fs);


        //


        int lsp(-1), fsp(-1), rsp(-1);
        if (STEP != pSTEP)
        {


            /*display_FRONT(0);
            display_LEFT(0);
            display_RIGHT(0);
            display_TURN(0);*/



            char wBuff[64];
            int wLeng = sprintf(wBuff, "%d\r\n", STEP); //       Ȳ    
            Serial.write(wBuff, wLeng);

            //      
            //Serial.println(x);
            //Serial.println(y);
            //Serial.println(z);
            //Serial.println(r_d);
            //Serial.println(l_d);
            //Serial.println(f_d);


            dv_mode = 1; // ӵ          on
            dv_chkTime = ct;// ӵ                ð       


            pSTEP = STEP;
            if (STEP != 0) {
                switch (STEP) {
                case 1:
                    nodeindex--;
                    donode(noderoute[nodeindex], noderoute[nodeindex]);
                    nodecheck = true;
                    break;
                default:
                    if (nodecheck) nodecheck = false;
                    else donode(STEP);
                }

                if (EEPROM.read(readindex) == 2) lsp = 100; // ls_on false
                else if (EEPROM.read(readindex) == 4) rsp = 100; // rs_on false
                else if (EEPROM.read(readindex) == 3) fsp = 50; // fs_on false
                else if (EEPROM.read(readindex) == 34) { rsp = 100; fsp = 50; }
                //else {lsp=-1;rsp=-1;fsp=-1;}
                readindex++;
            }
        }

        rs = analogRead(A0);
        ls = analogRead(A1);
        fs = analogRead(A2);

        if (rsp != -1)rs = rsp;
        if (lsp != -1)ls = lsp;
        if (fsp != -1)fs = fsp;

        //////////////// Ʃ      /////////////////

        //  ,  ,           ذ  ex)170      Ÿ            ν 
        bool rs_on = (rs >= 180);
        bool ls_on = (ls >= 180);
        bool fs_on = (fs >= 90);



        //   Ӹ       
        if (dv_mode == 1 && STEP == 0)
        {
            //Serial.print(ct);
            if (ave_velo(ct))
            {
                dv_mode = 0; //   Ӹ        
                //Serial.println(av);
                //Serial.print("off"); //      ˸  
                av = 250; //  ռӵ  250      缳   ( ʿ   Ʃ  )
                if ((fs <= 450) && (fs >= 300)) //                   ӵ       ( ʿ   Ʃ  )
                    av = 200;
                else if (fs <= 300) //          
                    av = 150;
            }

        }

        ////////////////////////////////////////

        //  Ȳ    Ǵ  
        if (STEP == 0 || STEP == 3)
        {

            chkTime = ct;

            if (!rs_on && !ls_on && !fs_on) //   +    +   
            {

                STEP = 1; //    
            }

            else if (!rs_on && !ls_on && fs_on) //  +    +    // б    üũ
            {
                STEP = 0;
                DISPLAY_(0);
                Linear(ct, ls, av); //    
            }

            else if (!rs_on && ls_on && (fs <= 150)) //  +  +  
            {

                STEP = 2; //        
            }

            else if (!rs_on && ls > 300 && fs_on) //  +  +  
            {

                //    
                STEP = 3;
                step3_chktime = ct;
                DISPLAY_(0);
                Linear(ct, b - rs - a, av); //                        ƴ    갪       ü 

            }
            else if (rs_on && !ls_on && !fs_on)
            {
                STEP = 4;
            }
            else if (rs_on && !ls_on && fs_on)
            {
                STEP = 4;
            }
            else if (rs_on && ls_on && !fs_on)
            {
                STEP = 4;
            }
            else if (rs_on && ls_on && fs_on)
            {
                STEP = 4;
            }



        }

        else if (STEP == 1)
        {
            DISPLAY_(1);
            if (U_turn(ct)) {
                m_step = STEP = 0;

            }
        }
        else if (STEP == 2)//  ȸ  
        {
            DISPLAY_(2);

            if (RL_Turn(ct, -1)) {
                m_step = STEP = 0;

            }
        }
        else if (STEP == 4)//  ȸ  
        {
            DISPLAY_(4);
            if (RL_Turn(ct, 1)) {
                m_step = STEP = 0;

            }
        }



        digitalWrite(12, rw < 0);
        digitalWrite(13, lw < 0);
        analogWrite(10, abs(rw));
        analogWrite(11, abs(lw));

        //if (mySerial.available()) {
        //  Serial.write(Serial.read());  //                   ø      Ϳ     
        //}
        //if (Serial.available()) {
        //  mySerial.write(Serial.read());  // ø                      ߽       WRITE
        //}
    }
    else {
        DISPLAY_(9);
        nodeindex = 0;
        readindex = 0;
    }
}
