#include "Odometry.h"

Odometry::Odometry(){

    preEncountX = 0;//過去値
    preEncountY = 0;//過去値
    preTheta = 0;

    lVeloX = 0;
    lVeloY = 0;
    gVeloX = 0;
    gVeloY = 0;
    gOmega = 0;

    gPosiX = 0;
    gPosiY = 0;
    gTheta = 0;

    offsetX = 0;
    offsetY = 0;
    offsetT = 0;
}

void Odometry::calculate(int _encountX, int _encountY, double _theta){

    // --<エンコーダの差>--
    int dEncountX = (_encountX - preEncountX);
    int dEncountY = (_encountY - preEncountY);

    // --<エンコーダ速度(mm/s)変換>--
    lVeloX = (double)dEncountX * (double)WHL_DIM * PI / INT_TIME / (double)QER;
    lVeloY = (double)dEncountY * (double)WHL_DIM * PI / INT_TIME / (double)QER;
    gOmega = (_theta - preTheta) / INT_TIME;

    // --<エンコーダの速度をローカルからグローバルへ>--(グローバル)--
    gTheta = _theta;
    gVeloX = lVeloX * cos(theta() - gOmega/2.0*INT_TIME) - lVeloY * sin(theta() - gOmega/2.0*INT_TIME);
    gVeloY = lVeloX * sin(theta() - gOmega/2.0*INT_TIME) + lVeloY * cos(theta() - gOmega/2.0*INT_TIME);

    // --<方向軸(x,y,rot)の距離(mm)変換>--(グローバル)
    gPosiX += gVeloX*INT_TIME;
    gPosiY += gVeloY*INT_TIME;

    preEncountX = _encountX;
    preEncountY = _encountY;
    preTheta = _theta;
}


double Odometry::posiX(){
    return gPosiX + offsetX;
}

double Odometry::posiY(){
    return gPosiY + offsetY;
}

double Odometry::theta(){
    return gTheta + offsetT;
}


double Odometry::veloX(){
    return lVeloX;
}

double Odometry::veloY(){
    return lVeloY;
}

double Odometry::omega(){
    return gOmega;
}


void Odometry::setPosition(double _gPosiX, double _gPosiY, double _theta){
    offsetX = _gPosiX - gPosiX;
    offsetY = _gPosiY - gPosiY;
    offsetT = _theta - gTheta;
}