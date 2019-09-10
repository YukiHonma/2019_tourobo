#include "Sequence.h"


Sequence::Sequence(){
    arrayNum = 0;
    seqNum = 0;
}

int Sequence::add(char sig, int t1, int t2, int t3){
    if(arrayNum >= MAX_DATANUM) return -1;

    points[arrayNum].signal = sig;
    points[arrayNum].target1 = t1;
    points[arrayNum].target2 = t2;
    points[arrayNum].target3 = t3;

    arrayNum++;

    return arrayNum;
}


char Sequence::nowSig(){
    return points[seqNum].signal;
}

int Sequence::nowPara(int target){
    switch(target){
        case 0:
            return points[seqNum].target1;
            break;
        case 1:
            return points[seqNum].target2;
            break;
        case 2:
            return points[seqNum].target3;
            break;
    }
    return -1;
}


int Sequence::set(int num){
    seqNum = num;
    return check();
}

int Sequence::next(){
    seqNum++;
    return check();
}
    
int Sequence::skip(int num){
    seqNum += num;
    return check();
}

int Sequence::skip(char sig){
    while(points[seqNum].signal != sig && seqNum < MAX_DATANUM){
        seqNum++;
    }
    return check();
}

bool Sequence::changed(){
    bool ret = changeFlag;
    changeFlag = false;
    return ret;
}


int Sequence::check(){
    changeFlag = true;
    if(seqNum >= MAX_DATANUM)return -1;
    return seqNum;
}
