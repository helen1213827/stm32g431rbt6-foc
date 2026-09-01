#ifndef __POSITION_LOOP_H
#define __POSITION_LOOP_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"


typedef struct 
{
    PI_t pi;
    float speed_ref;
    float kd;         /* 速度阻尼: 指令 = Kp*err - Kd*speed, 防冲过头 */
    float sm_cmd;     /* 平滑后的速度指令 (加速度限幅状态) */
    bool enabled;
} PositionLoop_t;


void PositionLoop_Init(PositionLoop_t *pl, float kp, float ki, float kd, float out_limit);
void PositionLoop_Run(PositionLoop_t *pl, float pos_ref, uint16_t enc_raw, float speed, float dt);
void PositionLoop_SetGains (PositionLoop_t *pl, float ki, float kp);
void PositionLoop_SetKd (PositionLoop_t *pl, float kd);
void PositionLoop_Reset(PositionLoop_t *pl);



#endif