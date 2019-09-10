#include "Filters.h"

LowPassFilter::LowPassFilter(double _T, double intTime){
    setParas(_T, intTime);
    preOutput = 0;
}

void LowPassFilter::setParas(double _T, double intTime){
    T = _T;
    time = intTime;
}

void LowPassFilter::putPre(double _preOutput){
    preOutput = _preOutput;
}

double LowPassFilter::filtering(double input){
    double output = (time*input + T*preOutput)/(T + time);
    preOutput = output;
    return output;
}


Limitter::Limitter(double accel){
    preInput = 0;
    setPara(accel);
}

void Limitter::setPara(double accel){
    maxAccel = accel;
}

void Limitter::putPre(double _preIntput){
    preInput = _preIntput;
}

double Limitter::filtering(double input){
    if( input > preInput + maxAccel ) input = preInput + maxAccel;
    if( input < preInput - maxAccel ) input = preInput - maxAccel;

    preInput = input;

    return input;
}