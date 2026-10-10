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

/* id_fb / iq_fb 必须传【瞬时值】(不要传低通滤波过的量),
 * 反馈上的滤波器会直接吃掉电流环带宽与相位裕度。
 * vd_ff / vq_ff 为 dq 解耦 + 反电动势前馈 (不需要时传 0)。 */
void CurrentLoop_Run(CurrentLoop_t *cl, float id_ref, float iq_ref,
float id_fb, float iq_fb, float vd_ff, float vq_ff, float dt);


void CurrentLoop_SetGains(CurrentLoop_t *cl,float d_kp,float d_ki, float q_kp,float q_ki );

void CurrentLoop_Reset(CurrentLoop_t *cl);
#endif