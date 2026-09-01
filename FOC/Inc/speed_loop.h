#ifndef __SPEED_LOOP_H
#define __SPEED_LOOP_H

#include <stdbool.h>
#include <stdint.h>
#include "pid.h"

typedef struct
{
    PI_t pi;
    float speed;
    float speed_raw;
    float iq_ref;
    float alpha;
    uint16_t last_enc;
    bool first_run;   /* 首采样保护: 避免 last_enc 初始 0 产生巨大假速度 */
    bool enabled;
} SpeedLoop_t;

void SpeedLoop_Init(SpeedLoop_t *sl, float kp, float ki, float out_limit, float lpf_alpha);
void SpeedLoop_Run(SpeedLoop_t *sl, float speed_ref, float enc_raw, float dt);
void SpeedLoop_SetGains(SpeedLoop_t *sl, float kp, float ki);
void SpeedLoop_Reset(SpeedLoop_t *sl);



#endif