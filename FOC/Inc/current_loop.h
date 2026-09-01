#ifndef __CURRENT_LOOP_H
#define __CURRENT_LOOP_H

#include "pid.h"
#include <stdbool.h>

typedef struct{
    PI_t pi_d;
    PI_t pi_q;
    float vq;
    float vd;
    bool enabled;
} CurrentLoop_t;


void CurrentLoop_Init(CurrentLoop_t *cl,
float kp, float ki, float out_limit);

void CurrentLoop_Run(CurrentLoop_t *cl, float id_ref, float iq_ref,
float id_fb, float iq_fb, float dt);


void CurrentLoop_SetGains(CurrentLoop_t *cl,float d_kp,float d_ki, float q_kp,float q_ki );

void CurrentLoop_Reset(CurrentLoop_t *cl);
#endif