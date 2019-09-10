#include "PIDControl.h"


PidControl::PidControl(double gainKP, double gainKI, double gainKD, double intTime){
    setParas(gainKP, gainKI, gainKD, intTime);
    clearInteg();
}


double PidControl::feedback(double error){
    static double preError = error;

    integ += (preError + error)*time/2.0;

    double output = kp*error + ki*integ + kd*(error - preError)/time;

    preError = error;

    return output;
}

void PidControl::setParas(double gainKP, double gainKI, double gainKD, double intTime){
    kp = gainKP;
    ki = gainKI;
    kd = gainKD;
    time = intTime;
}

void PidControl::clearInteg(){
    integ = 0;
}