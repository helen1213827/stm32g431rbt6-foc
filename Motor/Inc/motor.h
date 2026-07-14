#ifndef __MOTOR_H
#define __MOTOR_H

typedef struct
{
    float ia;
    float ib;
    float ic;

    float ialpha;
    float ibeta;

    float id;
    float iq;

    float vd;
    float vq;

    float theta;

    float dutyA;
    float dutyB;
    float dutyC;

    float valpha;
    float vbeta;

    float id_ref;
    float iq_ref;
}Motor_t;

extern Motor_t motor;

#endif // __MOTOR_H
