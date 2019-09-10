#include "Communication.h"


Communication::Communication(HardwareSerial *_comm){
    comm = _comm;
    for(int i=0; i<3; i++)dist[i] = 0;
    for(int i=0; i<MAX_BUFF; i++)gData[i] = 0;
    counter = 0;
}

void Communication::begin(long baudrate){
    comm->begin(baudrate);
}

int Communication::get(){
    int ret = 0;

    int n = comm->available();

    if(n == 0) return -1;

    for(int i=0; i<n; i++){
        gData[counter] = (char)comm->read();

        if((gData[counter] == (char)0xFF) && (counter > 6)){
            ret = 0;
            for(int i=0; i<3; i++){
                dist[i] = (((uint16_t)gData[counter-6 + i*2] << 7) | (uint16_t)gData[counter-5 + i*2]);
                ret += (dist[i] > 0);
            }
            counter = 0;
        }
        else if(counter >= MAX_BUFF){
            counter = 0;
        }
        else {
            counter++;
        }
    }

    return ret;
}
