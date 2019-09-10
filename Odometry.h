#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "defines.h"

class Odometry{
    public:

        Odometry();

        //自己位置を計算する
        void calculate(int encountX, int encountY, double theta);

        //グローバル座標でのx軸の位置を返す
        double posiX();

        //グローバル座標でのy軸の位置を返す
        double posiY();

        //ローカル座標系でのx方向速度
        double veloX();
        
        //ローカル座標系でのy方向速度
        double veloY();

        //角度を返す
        double theta();

        //角速度を返す
        double omega();

        //現在の位置座標を指定する
        void setPosition(double _gPosiX, double _gPosiY, double _theta);

    private:
        
        //過去値
        int preEncountX;
        int preEncountY;
        double preTheta;

        //local 速度
        double lVeloX;
        double lVeloY;

        //global 速度
        double gVeloX;
        double gVeloY;
        double gOmega;

        //global 位置
        double gPosiX;
        double gPosiY;
        double gTheta;

        double offsetX;
        double offsetY;
        double offsetT;
};

#endif