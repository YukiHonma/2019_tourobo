#include "MachineUnits.h"


UpUnit::UpUnit(int _pinSensor, int _pinAct){
    pinSensor = _pinSensor;
    pinAct = _pinAct;
    found = 0;
}


void UpUnit::init(){
    pinMode(pinSensor, INPUT_PULLUP);
    pinMode(pinAct, OUTPUT);
    act(0);
}

void UpUnit::act(bool state){
    if(state) digitalWrite(pinAct, HIGH);
    else      digitalWrite(pinAct, LOW);
    found = 0;
}

int UpUnit::watch(){
    return !digitalRead(pinSensor);
}

int UpUnit::passive(){
    if(found != 0) found++;
    else if(watch()){
        act(1);
        found++;
    }
    return found;
}


CBUnit::CBUnit(int _pinSensor, int _pinServo, int _angleOpen, int _angleClose) : Claw(){
    
    pinSensor = _pinSensor;
    pinServo = _pinServo;
    angleOpen = _angleOpen;
    angleClose = _angleClose;

    found = 0;
}

void CBUnit::init(){
    pinMode(pinSensor, INPUT_PULLUP);
    Claw.attach(pinServo);
    act(0);
}


void CBUnit::act(bool state){
    if(state) Claw.write(angleOpen);
    else      Claw.write(angleClose);
    found = 0;
}

int CBUnit::watch(){
    return !digitalRead(pinSensor);
}

int CBUnit::passive(){
    if(watch()){
        act(0);
        return 1;
    }
    else{
        act(1);
    }
    return 0;
}

bool CBUnit::set(){
    return Claw.attached();
}


Omni4W::Omni4W(double wheelLength) : MD(&Serial2, 1){
    wl = wheelLength;
}

void Omni4W::begin(){
    MD.begin(115200);
}

void Omni4W::move(double vx, double vy, double omega){

    if(!allowFlag) return;

    double vm[4] = {};

    vm[0] =  vx*_SQRT2 + vy*_SQRT2 + omega*wl;
    vm[1] = -vx*_SQRT2 + vy*_SQRT2 + omega*wl;
    vm[2] =  vx*_SQRT2 - vy*_SQRT2 + omega*wl;
    vm[3] = -vx*_SQRT2 - vy*_SQRT2 + omega*wl;

    // v/(wheelDim*PI)*QuadratureEncRes = QPPS
    MD.SpeedM1M2(ADR_MD1, (int)(vm[0]*M_QER/(M_WHL_DIM*PI)), (int)(vm[1]*M_QER/(M_WHL_DIM*PI)));
    MD.SpeedM1M2(ADR_MD2, (int)(vm[2]*M_QER/(M_WHL_DIM*PI)), (int)(vm[3]*M_QER/(M_WHL_DIM*PI)));
    
}

void Omni4W::stop(){
    allowFlag = false;

    MD.ForwardM1(ADR_MD1,0);
    MD.ForwardM2(ADR_MD1,0);
    MD.ForwardM1(ADR_MD2,0);
    MD.ForwardM2(ADR_MD2,0);
}

void Omni4W::allow(){
    allowFlag = true;
}