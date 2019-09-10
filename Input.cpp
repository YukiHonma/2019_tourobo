#include "Input.h"


SwitchIn::SwitchIn(uint8_t _pinName){
    pinName = _pinName;
    pinMode(pinName, INPUT);
    preRead = read();
}

SwitchIn::SwitchIn(uint8_t _pinName, uint8_t _Mode){
    pinName = _pinName;
    pinMode(pinName, _Mode);
    preRead = read();
}

int SwitchIn::state(){
    int pinRead = read();
    int ret = 0;

    if(pinRead) ret += 2;
    if(preRead) ret -= 1;

    preRead = pinRead;

    return ret;
}

int SwitchIn::read(){
    return digitalRead(pinName);
}
