#ifndef SEQUENCE_H
#define SEQUENCE_H

#include "FileManipulator.h"
#include "defines.h"


typedef struct {
    char signal;
    int target1;
    int target2;
    int target3;
} pointData;

class Sequence{
public:
    Sequence();

    pointData points[MAX_DATANUM];

    char nowSig();
    int nowPara(int target);

    int add(char sig, int t1 = 0, int t2 = 0, int t3 = 0);


    int next();

    int set(int num);
    int skip(int num);
    int skip(char sig);

    bool changed();
private:

    int arrayNum;
    int seqNum;
    bool changeFlag;

    int check();

};


#endif