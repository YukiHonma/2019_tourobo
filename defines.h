#ifndef DEFINES_H
#define DEFINES_H

// controller defines

#define INT_TIME 0.01

#define ADR_MD1 128
#define ADR_MD2 129

#define PIN_BD_SENS 36
#define PIN_CBR_SENS 46
#define PIN_CBC_SENS 45
#define PIN_CBL_SENS 44
#define PIN_BD_ARM 11
#define PIN_CBR_SERVO 41
#define PIN_CBC_SERVO 42
#define PIN_CBL_SERVO 43

#define PIN_SORE_UPSIDE 12
#define PIN_SORE_UPCENT 10
#define PIN_SORE_FLIP 13

#define PIN_CONT_X A0
#define PIN_CONT_Y A1
#define PIN_CONT_T A2

#define PIN_PRC_SW A3
#define PIN_FLD_SW A4
#define PIN_RTY_SW A5

#define PIN_LED_RED 9
#define PIN_LED_BLUE 8

#define PIN_USER_SW 22
#define PIN_USER_LED 23


#define FILE_SEQ_RED "R.txt"
#define FILE_SEQ_BLUE "B.txt"

#define MAX_DATANUM 80


// parameters


#define ACCEL 2800.00
#define ALPHA 4.772

#define MAX_SPEED 2800.00
#define MAX_ANG 4.772

#define CONT_SPEED 1400.00
#define CONT_ANG 3.1415

#define ADJ_SPEED 800.00
#define ADJ_ANG 1.5772


//  position type PID  gains
#define KPxp 1.24
#define KIxp 0.00
#define KDxp 0.00

#define KPyp 1.24
#define KIyp 0.00
#define KDyp 0.00

#define KPth 1.07
#define KIth 0.00
#define KDth 0.000


//  velocity type PD  gains
#define KPxv 1.83
#define KDxv 0.00

#define KPyv 1.83
#define KDyv 0.00

#define KPom 1.62
#define KDom 0.00


// mechanical defines

#define QER 800
#define WHL_DIM 48

#define M_QER 2048
#define M_WHL_DIM 100

#define OPEN_ANGLE 150
#define CLOSE_ANGLE 80

#define UNIT_ACT_LAG 5

#define WHL_DST 310.0

#define TOF_INTERVAL 550


// numerical defines

#define SQRT2 1.41421356
#define _SQRT2 0.707106781




#endif