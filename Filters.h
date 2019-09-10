#ifndef FILTERS_H
#define FILTERS_H

class LowPassFilter{
public:
    LowPassFilter(double _T, double intTime);
    
    void setParas(double _T, double intTime);
    void putPre(double _preOutput);
    double filtering(double input);

private:
    double T, time;
    double preOutput;
};


class Limitter{
public:
    Limitter(double accel);

    void setPara(double accel);
    void putPre(double _preIntput);
    double filtering(double input);

private:
    double preInput;
    double maxAccel;
};


#endif