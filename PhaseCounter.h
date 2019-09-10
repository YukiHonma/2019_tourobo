#ifndef PHASECOUNTER_H
#define PHASECOUNTER_H

#include "iodefine_gcc63n.h"

#define ENC_INIT_VAL 0x7FFF

class PhaseCounter
{
public:
    PhaseCounter(int);

    int getCount();
    void init();

private:
    bool ch_available;
    bool init_done;
    
    unsigned short int pre_rawcount;
    int encount;
    
    int g_ch;
};


#endif