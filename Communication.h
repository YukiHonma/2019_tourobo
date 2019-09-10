#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <HardwareSerial.h>


#define MAX_BUFF 20

class Communication{
public:
    Communication(HardwareSerial *_comm);

    void begin(long baudrate);
    int get();

    uint16_t dist[3];
    char gData[MAX_BUFF];

    HardwareSerial *comm;
private:
    unsigned short counter;
};



#endif