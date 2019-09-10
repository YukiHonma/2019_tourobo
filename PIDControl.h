#ifndef PIDCONTROL_H
#define PIDCONTROL_H


class PidControl
{
public:
    PidControl(double gainKP, double gainKI, double gainKD, double intTime);

    double feedback(double error);
    void setParas(double gainKP, double gainKI, double gainKD, double intTime);
    void clearInteg();

private:
    double kp,ki,kd,time;
    double integ;
};


#endif