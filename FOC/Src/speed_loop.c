#include "speed_loop.h"
#include "pid.h"
#include <stdbool.h>

/* 在线调参安全限幅: 防止误设巨大/负增益导致跑飞 */
#define SPD_KP_MAX  2.0f
#define SPD_KI_MAX  0.5f

void SpeedLoop_Init(SpeedLoop_t *sl, float kp, float ki, float out_limit, float lpf_alpha)
{
    PI_Init(&sl->pi, kp, ki, -out_limit, out_limit);
    sl->speed = 0.0f;
    sl->speed_raw = 0.0f;
    sl->iq_ref = 0.0f;
    sl->alpha = lpf_alpha;
    sl->last_enc = 0;
    sl->first_run = true;
    sl->enabled = true;

}

void SpeedLoop_Run(SpeedLoop_t *sl, float speed_ref, float enc_raw, float dt)
{
    /* 首采样: 只记录起始位置, 不计算速度 */
    if (sl->first_run)
    {
        sl->last_enc = enc_raw;
        sl->first_run = false;
        sl->iq_ref = 0.0f;
        return;
    }

    int32_t  delta = (int32_t)enc_raw - (int32_t)sl->last_enc;
    if      (delta >  32767) delta -= 65536;
    else if (delta < -32768) delta += 65536;
    if (delta >= -3000 && delta <= 3000) sl->last_enc = enc_raw;

    sl->speed_raw = delta *(6.283185f/65536.0f)/dt;

    sl->speed = sl->speed * (1.0f - sl->alpha) + sl->speed_raw * sl->alpha;

    if (!sl->enabled) { sl->iq_ref = 0.0f; return; }
    sl->iq_ref = PI_Run(&sl->pi, speed_ref, sl->speed, dt);
}

void SpeedLoop_SetGains(SpeedLoop_t *sl, float kp, float ki)
{
    /* -1 (或任何负值) 表示"这一项不改" */
    if (kp >= 0.0f) sl->pi.kp = (kp > SPD_KP_MAX) ? SPD_KP_MAX : kp;
    if (ki >= 0.0f) sl->pi.ki = (ki > SPD_KI_MAX) ? SPD_KI_MAX : ki;
}

void SpeedLoop_Reset(SpeedLoop_t *sl)
{
    sl->pi.integral = 0;
    sl->iq_ref = 0;
    sl->last_enc = 0;
    sl->first_run = true;
}