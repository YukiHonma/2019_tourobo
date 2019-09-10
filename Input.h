#ifndef INPUT_H
#define INPUT_H

#include <Arduino.h>

class SwitchIn{
public:
    SwitchIn(uint8_t _pinName);
    SwitchIn(uint8_t _pinName, uint8_t _Mode);

    //@return 1:HIGH, 2:EDGE_UP, 0:LOW, -1:EDGE_DOWN

    int state();
    int read();

private:
    uint8_t pinName;
    uint8_t preRead;
};



#endif