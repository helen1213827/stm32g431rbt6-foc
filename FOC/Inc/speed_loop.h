#ifndef __SPEED_LOOP_H
#define __SPEED_LOOP_H

#include <stdbool.h>
#include <stdint.h>
#include "pid.h"

typedef struct
{
    PI_t pi;
    float speed;      /* 滤波后的机械角速度 (rad/s), 供打印 */
    float speed_raw;  /* 本周期输入的估计速度 (rad/s) */
    float iq_ref;
    bool enabled;
} SpeedLoop_t;

/* 速度反馈由调用方算好后以 rad/s 传入 (见 foc.c 的 1kHz 滑窗测速),
 * 速度环不再自己做编码器差分 + 滤波 —— 这样测速算法与环路解耦。 */
void SpeedLoop_Init(SpeedLoop_t *sl, float kp, float ki, float out_limit);
void SpeedLoop_Run(SpeedLoop_t *sl, float speed_ref, float speed_fb, float dt);
void SpeedLoop_SetGains(SpeedLoop_t *sl, float kp, float ki);
void SpeedLoop_SetLimit(SpeedLoop_t *sl, float out_limit);   /* 输出限幅 = 最大 iq (A) */
void SpeedLoop_Reset(SpeedLoop_t *sl);



#endif