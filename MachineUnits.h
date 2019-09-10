//#include <RoboClaw.h>

#ifndef MACHINEUNITS_H
#define MACHINEUNITS_H

#include <Arduino.h>
#include <servo.h>
#include "RoboClaw.h"
#include "defines.h"


class UpUnit
{
public:
    // Constructor.
    // @param pinName :sensor input, pinName :arm output
    UpUnit(int _pinSensor, int _pinArm);

    // Initialization.
    void init();

    // Arm Unit.
    // @param 1 :raise, 0 :falldown
    void act(bool state);

    // Sensor.
    // @return 1 :detected, 0 :undetected
    int watch();

    // Interlocking.
    int passive();

    int pinSensor;
    int pinAct;
private:
    int found;
};

class CBUnit{
public:
    CBUnit(int _pinSensor, int _pinArm, int _angleOpen, int _angleClose);

    //void initialize(int _pinSensor, int _pinServo);
    void init();
    void act(bool state);
    int watch();
    int passive();
    bool set();


    Servo Claw;

private:
    int pinServo;
    int pinSensor;

    int found;
    int angleOpen;
    int angleClose;

};


class Omni4W
{
public:
    Omni4W(double wheelLength);

    // in rectangular coordinate system
    //@param double vx[mm/s], double vy[mm/s], omega[rad/s]
    void move(double vx, double vy, double omega);

    //begin communication with MDs.
    void begin();

    // stop all motors. ban to move motors.
    void stop();

    // allow to move motors.
    void allow();

private:
    double wl;
    RoboClaw MD;
    bool allowFlag;
};



#endif
