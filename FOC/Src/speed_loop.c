#include "speed_loop.h"
#include "pid.h"
#include <stdbool.h>

/* 在线调参安全限幅: 防止误设巨大/负增益导致跑飞。
 * Ki 是连续域积分增益 (integral += err*ki*dt), 速度环量级约 0.01~5,
 * 原来的上限 0.5 会限制调参范围。 */
#define SPD_KP_MAX  20.0f
#define SPD_KI_MAX  50.0f

void SpeedLoop_Init(SpeedLoop_t *sl, float kp, float ki, float out_limit)
{
    PI_Init(&sl->pi, kp, ki, -out_limit, out_limit);
    sl->speed = 0.0f;
    sl->speed_raw = 0.0f;
    sl->iq_ref = 0.0f;
    sl->enabled = true;
}

void SpeedLoop_Run(SpeedLoop_t *sl, float speed_ref, float speed_fb, float dt)
{
    sl->speed_raw = speed_fb;
    sl->speed     = speed_fb;

    if (!sl->enabled) { sl->iq_ref = 0.0f; return; }
    sl->iq_ref = PI_Run(&sl->pi, speed_ref, sl->speed, dt);
}

void SpeedLoop_SetGains(SpeedLoop_t *sl, float kp, float ki)
{
    /* -1 (或任何负值) 表示"这一项不改" */
    if (kp >= 0.0f) sl->pi.kp = (kp > SPD_KP_MAX) ? SPD_KP_MAX : kp;
    if (ki >= 0.0f) sl->pi.ki = (ki > SPD_KI_MAX) ? SPD_KI_MAX : ki;
}

/* 设置速度环输出限幅 (即最大 iq 指令, A)。
 * 这一层直接决定机器能给出多大加速度: a_max = Kt/J × i_max。
 * 参考轨迹的加速度必须小于 a_max, 否则环路必然饱和 → 极限环。 */
void SpeedLoop_SetLimit(SpeedLoop_t *sl, float out_limit)
{
    if (out_limit <= 0.0f) return;

    sl->pi.out_max =  out_limit;
    sl->pi.out_min = -out_limit;

    /* 积分限幅不能超过输出限幅, 否则积分单独就能把输出顶满 */
    if (sl->pi.int_limit >  out_limit) sl->pi.int_limit =  out_limit;
    if (sl->pi.int_limit < -out_limit) sl->pi.int_limit = -out_limit;
}

void SpeedLoop_Reset(SpeedLoop_t *sl)
{
    sl->pi.integral = 0;
    sl->iq_ref = 0;
}