#include <Arduino.h>

#include "PhaseCounter.h"
#include "defines.h"
#include "MachineUnits.h"
#include "MSTimerTPU3.h"
#include "LPMS_ME1.h"
#include "PIDControl.h"
#include "Filters.h"
//#include "FileManipulator.h"
#include "Sequence.h"
#include "Input.h"
#include "Odometry.h"
#include "Communication.h"

PhaseCounter Enc1(1);
PhaseCounter Enc2(2);

UpUnit BD(PIN_BD_SENS, PIN_BD_ARM);
CBUnit CBR(PIN_CBR_SENS, PIN_CBR_SERVO, 110, 155);
CBUnit CBC(PIN_CBC_SENS, PIN_CBC_SERVO, 130, 84);
CBUnit CBL(PIN_CBL_SENS, PIN_CBL_SERVO, 101, 50);
Omni4W Movement(WHL_DST);


Lpms_me1 gyro(&Serial3);

SwitchIn ProcessSW(PIN_PRC_SW);
SwitchIn FieldSW(PIN_FLD_SW);
SwitchIn RetrySW(PIN_RTY_SW);

PidControl Posi[3] = {
    PidControl(KPxp, KIxp, KDxp, INT_TIME), 
    PidControl(KPyp, KIyp, KDyp, INT_TIME), 
    PidControl(KPth, KIth, KDth, INT_TIME)
};

PidControl Velo[3] = {
    PidControl(KPxv, 0, KDxv, INT_TIME), 
    PidControl(KPyv, 0, KDyv, INT_TIME), 
    PidControl(KPom, 0, KDom, INT_TIME)
};

LowPassFilter LPF[3] = {
    LowPassFilter((ACCEL/MAX_SPEED), INT_TIME),
    LowPassFilter((ACCEL/MAX_SPEED), INT_TIME),
    LowPassFilter((ACCEL/MAX_SPEED), INT_TIME)
};

Limitter LMF[3] = {
    Limitter(ACCEL*INT_TIME),
    Limitter(ACCEL*INT_TIME),
    Limitter(ALPHA*INT_TIME)
};


//FileManipulator FileData;
Sequence Order;
Odometry Odm;

Communication ToFs(&Serial1);

int encount = 0, encount2 = 0;

bool interrupted = false;
bool light = false;

bool field = false;//false:blue, true:red

/*
template<typename T_n>
T_n limit(T_n num, T_n maxim, T_n minim){
         if(num > maxim) return maxim;
    else if(num < minim) return minim;
    else                 return num;
}*/

double limit(double num, double maxim, double minim){
    if(num > maxim) return maxim;
    if(num < minim) return minim;
    return num;
}

int limit(int num, int maxim, int minim){
    if(num > maxim) return maxim;
    if(num < minim) return minim;
    return num;
}

double cut(double num, double uLim, double lLim){
    if(num < uLim && num > lLim) return 0;
    return num;
}

void LEDout(uint8_t binary){
    digitalWrite(PIN_LED0, binary & 0b0001);
    digitalWrite(PIN_LED1, binary & 0b0010);
    digitalWrite(PIN_LED2, binary & 0b0100);
    digitalWrite(PIN_LED3, binary & 0b1000);
}

void LEDerror(uint8_t binary){
    while(1){
        LEDout(binary);
        delay(200);
        LEDout(0);
        delay(200);
    }
}

void setSequence(){

    //CB回収
    Order.add('S');
    Order.add('I');
    Order.add('B');
    Order.add('O', 0, 0, 0);
    Order.add('G');
    Order.add('C');
    Order.add('H');
    Order.add('A');
    
    //持ち上げ
    Order.add('R');
    Order.add('N');
    Order.add('W', 20);

    //BD回収位置
    if(field){ Order.add('P', 850, 2300, 0); }
    else{      Order.add('P', 850, -2300, 0); }
    Order.add('B');
    Order.add('C');
    Order.add('B');
    Order.add('U');
    Order.add('N');
    Order.add('W', 120);

    //BD設置位置
    if(field){ Order.add('P', 1260, 2650, -90); }
    else{      Order.add('P', 1260, -2650, 90); }
    Order.add('B');
    Order.add('C');
    Order.add('B');
    Order.add('N');
    Order.add('u');
    Order.add('W', 30);
    
    //退避 チェンジゾーン

    if(field){ Order.add('V', 0, -500, 0); }
    else{      Order.add('V', 0, 500, 0); }
    Order.add('W', 100);
    if(field){ Order.add('P', 1000, 4800, -90); }
    else{      Order.add('P', 1000, -4800, 90); }
    Order.add('W', 5); // Eとセット 重要
    Order.add('E', 300, 300, 5);
    Order.add('U');
    if(field){ Order.add('P', 2100, 4800, -90); }
    else{      Order.add('P', 2100, -4800, 90); }
    Order.add('B');
    Order.add('N');
    Order.add('C');
    Order.add('g');
    Order.add('B');
    Order.add('N');

    //BD渡り
    Order.add('U');
    Order.add('R');
    Order.add('h');
    Order.add('W', 5);
    Order.add('B');
    // Order.add('P', 700, 0, 0);
    // Order.add('W', 5);
    // Order.add('E', 50, 20, 2);
    // Order.add('V', 600, 0, 20);
    // Order.add('W', 320);
    //  Order.add('V', 800, 0, 22);
    Order.add('V', 800, 0, 25);
    Order.add('W', 400);

    Order.add('V', 0, 0, 3);
    //  Order.add('V', 0, 0, 0);
    Order.add('W', 60);
    Order.add('N');
    Order.add('W', 50);
    Order.add('O', 0, 0, 0);

    // ゴール位置合わせ
    if(field){ Order.add('P', 50, 740, 0); }
    else {     Order.add('P', 50, -740, 0); }

    Order.add('D', 0, 0, 0);
    Order.add('W', 10);
    Order.add('E', 20, 20, 1);
    Order.add('N');
    Order.add('T');
    Order.add('W', 20);
    Order.add('F');
    Order.add('W', 50);
    Order.add('f');
    Order.add('r');

    if(Order.add('Z') == -1) LEDerror(0b1111);
}

void interTimer(){
    encount = -Enc1.getCount();
    encount2= Enc2.getCount();

    interrupted = true;
}

void setup(){

    //config
    pinMode(PIN_LED0, OUTPUT);
    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_LED3, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_BLUE, OUTPUT);
    pinMode(PIN_USER_LED, OUTPUT);
    pinMode(PIN_SORE_UPSIDE, OUTPUT);
    pinMode(PIN_SORE_UPCENT, OUTPUT);
    pinMode(PIN_SORE_FLIP, OUTPUT);

    pinMode(PIN_PRC_SW, INPUT);
    pinMode(PIN_FLD_SW, INPUT);
    pinMode(PIN_RTY_SW, INPUT);
    
    analogReference(RAW12BIT);

    CBC.init();
    CBR.init();
    CBL.init();
    BD.init();

    //init
    LEDout(0b0001);
    digitalWrite(PIN_USER_LED, HIGH);
    digitalWrite(PIN_SORE_UPSIDE, LOW);
    digitalWrite(PIN_SORE_UPCENT, LOW);
    digitalWrite(PIN_SORE_FLIP, LOW);
    BD.act(0);

    delay(10);

    Movement.begin();
    Serial.begin(230400);
    ToFs.begin(115200);

    delay(10);

    if(gyro.init() == 0){ LEDout(0b0011); }
    else{                 LEDerror(0b0011); }

    do{
        field = (bool)FieldSW.read();
        digitalWrite(PIN_LED_RED, field);
        digitalWrite(PIN_LED_BLUE, !field);

        delay(10);
    }while(ProcessSW.state() != -1);
    
    setSequence();

    for(int i=0; i<3; i++){
        digitalWrite(PIN_LED_RED, LOW);
        digitalWrite(PIN_LED_BLUE, LOW);
        delay(100);
        digitalWrite(PIN_LED_RED, field);
        digitalWrite(PIN_LED_BLUE, !field);
        delay(100);
    }

    Enc1.init();
    Enc2.init();
    
    Movement.allow();

	MsTimerTPU3::set((int)(INT_TIME * 1000), interTimer); // 10ms period
	MsTimerTPU3::start();

    LEDout(0b0111);

}

void loop(){

    if(interrupted){
        static int count = 0;
        if(count == 100){
            digitalWrite(PIN_USER_LED, light);
            count = 0;
            light = !light;
        }
        count++;

        // sequence ////////////////////////////////////////////////

        Odm.calculate(encount, encount2, gyro.get_z_angle());
        Serial.print(ToFs.get());
        Serial.print('\t');

        static int moveMode = 0;

        static double refPosi[3] = {};
        static double refVelo[3] = {};
        static double cmd[3] = {};
        static double errPosi[3] = {};
        //static double errVelo[3] = {};

        static unsigned int wCounter = 0;


        do{
            
            switch(Order.nowSig()){

                /**********************
                    Robot process
                **********************/
                case 'S':
                    Order.next();
                    break;
                case 'W':
                    wCounter++;
                    if(wCounter == Order.nowPara(0)){
                        wCounter = 0;
                        Order.next();
                    }
                    break;
                case 'B':
                    if(ProcessSW.state() == -1)Order.next();
                    break;
                case 'I':
                    BD.act(0);
                    CBC.act(0);
                    CBR.act(0);
                    CBL.act(0);
                    Order.next();
                    break;
                case 'X':
                case 'Z':
                    moveMode = 0;
                    break;
                case 'L':
                    //led
                    LEDout(Order.nowPara(0));
                    Order.next();
                    break;
                case 'G':
                    if(!RetrySW.read()) Order.skip('g');
                    else Order.next();
                    break;
                case 'g':
                    //リトライ後の設定
                    if(!RetrySW.read()) Odm.setPosition((double)Order.nowPara(0), (double)Order.nowPara(1), (double)Order.nowPara(2));
                    Order.next();
                    break;

                /******************
                    under body
                ******************/

                case 'N':
                    moveMode = 0;
                    for(int i=0;i<3;i++){
                        refVelo[i] = 0;
                    }
                    Order.next();
                    break;
                case 'P':
                    for(int i=0;i<3;i++){
                        refPosi[i] = (double)Order.nowPara(i);  //位置目標設定
                        LMF[i].putPre(refVelo[i]);
                    }
                    refPosi[2] *= PI/180;
                    moveMode = 1;
                    Order.next();
                    break;
                case 'D':
                    // for(int i=0;i<3;i++){
                    //     refPosi[i] = (double)Order.nowPara(i);  //壁距離目標設定
                    //     LMF[i].putPre(refVelo[i]);
                    // }
                    // refPosi[2] *= PI/180;
                    // moveMode = 2;
                    if(field) { refPosi[1] -= ToFs.dist[1]; }
                    else { refPosi[1] += ToFs.dist[1]; }
                    Order.next();
                    break;
                case 'E':
                    if(abs((int)errPosi[0]) < Order.nowPara(0) && abs((int)errPosi[1]) < Order.nowPara(1) && abs(errPosi[2]) < (double)Order.nowPara(2)*PI/180.0) Order.next();
                    break;
                case 'C':
                    for(int i=0;i<3;i++){
                        LMF[i].putPre(refVelo[i]);
                    }
                    moveMode = 3;
                    Order.next();
                    break;
                case 'V':
                    for(int i=0; i<3; i++){
                        LPF[i].putPre(cmd[i]);
                        refVelo[i] = Order.nowPara(i);
                    } //一定移動値設定
                    refVelo[2] *= PI/180.0;
                    moveMode = 4;
                    Order.next();
                    break;
                case 'O':
                    Odm.setPosition((double)Order.nowPara(0), (double)Order.nowPara(1), (double)Order.nowPara(2)*PI/180.0); //自己位置値設定
                    Order.next();
                    break;
                

                /*****************
                    Upper body
                *****************/

                case 'H':
                    CBC.act(1);
                    CBR.act(1);
                    CBL.act(1);
                    Order.next();
                    break;
                case 'h':
                    CBC.act(0);
                    CBR.act(0);
                    CBL.act(0);
                    Order.next();
                    break;
                case 'A':
                    if( (CBC.passive() * CBR.passive() * CBL.passive()) && (ProcessSW.state() == -1) ){ Order.next(); }
                    break;
                case 'U':
                    BD.act(1);
                    Order.next();
                    break;
                case 'u':
                    BD.act(0);
                    Order.next();
                    break;
                case 'Q':
                    BD.passive();
                    Order.next();
                    break;
                case 'R':
                    digitalWrite(PIN_SORE_UPSIDE, HIGH);
                    digitalWrite(PIN_SORE_UPCENT, HIGH);
                    Order.next();
                    break;
                case 'r':
                    digitalWrite(PIN_SORE_UPSIDE, LOW);
                    digitalWrite(PIN_SORE_UPCENT, LOW);
                    Order.next();
                    break;
                case 'T':
                    digitalWrite(PIN_SORE_UPSIDE, HIGH);
                    digitalWrite(PIN_SORE_UPCENT, LOW);
                    Order.next();
                    break;
                case 'F':
                    digitalWrite(PIN_SORE_FLIP, HIGH);
                    Order.next();
                    break;
                case 'f':
                    digitalWrite(PIN_SORE_FLIP, LOW);
                    Order.next();
                    break;
            }

        }while(Order.changed());


        //足回り系
        switch(moveMode){
            case 0:   //停止
                for(int i=0;i<3;i++){
                    cmd[i] = 0;
                }
                break;

            case 1:   //接地 Odometry による feedback

                //local
                errPosi[2] = refPosi[2] - Odm.theta();
                refVelo[2] = LMF[2].filtering( limit( Posi[2].feedback(errPosi[2]), MAX_ANG, -MAX_ANG) );
                cmd[2] += Velo[2].feedback(refVelo[2] - Odm.omega()) * INT_TIME;

                errPosi[0] = (refPosi[0] - Odm.posiX()) *  cos(Odm.theta()) + (refPosi[1] - Odm.posiY()) * sin(Odm.theta());
                errPosi[1] = (refPosi[0] - Odm.posiX()) * -sin(Odm.theta()) + (refPosi[1] - Odm.posiY()) * cos(Odm.theta());

                refVelo[0] = LMF[0].filtering( limit( Posi[0].feedback(errPosi[0]), MAX_SPEED, -MAX_SPEED) );
                refVelo[1] = LMF[1].filtering( limit( Posi[1].feedback(errPosi[1]), MAX_SPEED, -MAX_SPEED) );

                cmd[0] += Velo[0].feedback(refVelo[0] - Odm.veloX()) * INT_TIME;
                cmd[1] += Velo[1].feedback(refVelo[1] - Odm.veloY()) * INT_TIME;

                break;

            case 2:   //ToF Sensorによるfeedback

                if(field){
                    errPosi[2] = refPosi[2] - Odm.theta();
                    errPosi[1] = -(refPosi[1] - ToFs.dist[1]);
                }
                else{
                    errPosi[2] = refPosi[2] - Odm.theta();
                    errPosi[1] = refPosi[1] - ToFs.dist[1];
                }

                errPosi[0] = refPosi[0] - ToFs.dist[2]*cos(errPosi[2]);

                refVelo[0] = LMF[0].filtering( limit( Posi[0].feedback(errPosi[0]), ADJ_SPEED, -ADJ_SPEED) );
                refVelo[1] = LMF[1].filtering( limit( Posi[1].feedback(errPosi[1]), ADJ_SPEED, -ADJ_SPEED) );
                refVelo[2] = LMF[2].filtering( limit( Posi[2].feedback(errPosi[2]), ADJ_ANG,   -ADJ_ANG) );

                cmd[0] += Velo[0].feedback(refVelo[0] - Odm.veloX()) * INT_TIME;
                cmd[1] += Velo[1].feedback(refVelo[1] - Odm.veloY()) * INT_TIME;
                cmd[2] += Velo[2].feedback(refVelo[2] - Odm.omega()) * INT_TIME;

                break;

            case 3:   //コントローラ

                refVelo[0] = LMF[0].filtering( cut(((double)analogRead(PIN_CONT_X) - 2048.0), 200, -200)*CONT_SPEED / 4096.0 );
                refVelo[1] = LMF[1].filtering( cut(((double)analogRead(PIN_CONT_Y) - 2048.0), 200, -200)*CONT_SPEED / 4096.0 );
                refVelo[2] = LMF[2].filtering( cut(((double)analogRead(PIN_CONT_T) - 2048.0), 200, -200)*CONT_ANG / 4096.0 );

                cmd[0] += Velo[0].feedback(refVelo[0] - Odm.veloX()) * INT_TIME;
                cmd[1] += Velo[1].feedback(refVelo[1] - Odm.veloY()) * INT_TIME;
                cmd[2] += Velo[2].feedback(refVelo[2] - Odm.omega()) * INT_TIME;

                break;
                
            case 4:   //一定値
                //値はシーケンスで入れている
                for(int i=0;i<3;i++){
                    //cmd[i] = LPF[i].filtering(refVelo[i]);
                    cmd[i] = refVelo[i];
                }
                break;

        }
        
        cmd[0] = limit(cmd[0], MAX_SPEED, -MAX_SPEED);
        cmd[1] = limit(cmd[1], MAX_SPEED, -MAX_SPEED);
        cmd[2] = limit(cmd[2], MAX_ANG, -MAX_ANG);
        
        Movement.move(cmd[0], cmd[1], cmd[2]);
        interrupted = false;
        
        //Serial.println(Order.nowSig());
        
        Serial.print(ToFs.dist[0]);
        Serial.print('\t');
        Serial.print(ToFs.dist[1]);
        Serial.print('\t');
        Serial.println(ToFs.dist[2]);
        
    }
}