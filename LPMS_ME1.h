#ifndef LPMS_ME1_H
#define LPMS_ME1_H

#include <Arduino.h>
#include "iodefine_gcc63n.h"

class Lpms_me1{
    public:
    Lpms_me1(HardwareSerial*);
    void goto_command_mode();
    void set_transmit_data();
    void set_filter_mode();
    void set_offset();
    void reset_orientation_offset();
    void get_sensor_data();
    float get_z_angle();  
    int recv_proc(int);
    int init();

    private:
    HardwareSerial* serial;
    byte buffer[25];
    float anglex, angley, anglez;
    float pre_rawanglex, pre_rawangley, pre_rawanglez;
    bool init_ignore;
};


#endif