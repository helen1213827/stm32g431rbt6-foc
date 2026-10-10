#include "current_loop.h"
#include "pid.h"

/* 在线调参安全限幅: 防止误设巨大/负增益导致跑飞。
 * 注意: Ki 是连续域积分增益 (integral += err*ki*dt),
 * 正确值约为 ωc·R, 对有刷小电机的量级在 1e3~1e4,
 * 原来的 CUR_KI_MAX=200 会把整定好的 Ki 直接截断。 */
#define CUR_KP_MAX  200.0f
#define CUR_KI_MAX  200000.0f

void CurrentLoop_Init(CurrentLoop_t *cl,
float kp, float ki, float out_limit)
{
    PI_Init(&cl->pi_d,kp,ki,-out_limit,out_limit);
    PI_Init(&cl->pi_q, kp, ki, -out_limit, out_limit);
    cl->vd = 0.0f;
    cl->vq = 0.0f;
    cl->enabled = true;
}

void CurrentLoop_Run(CurrentLoop_t *cl, float id_ref, float iq_ref,
float id_fb, float iq_fb, float vd_ff, float vq_ff, float dt)
{
    if (!cl->enabled)  { cl->vd = 0.0f; cl->vq = 0.0f; return;}

    /* PI 只负责残差; 主要负担 (反电动势 + dq 交叉耦合) 由前馈承担,
     * 这样 Ki 不必靠"硬顶"来消除稳态误差, 也就能调得小且稳。
     * 注意 PI 的抗饱和限幅仍按 PI 输出考核, 前馈不会把积分顶满。 */
    cl->vd = PI_Run(&cl->pi_d, id_ref, id_fb, dt) + vd_ff;
    cl->vq = PI_Run(&cl->pi_q, iq_ref, iq_fb, dt) + vq_ff;
}

void CurrentLoop_SetGains(CurrentLoop_t *cl,float d_kp,float d_ki, float q_kp,float q_ki )
{
    /* -1 (或任何负值) 表示"这一项不改" */
    if (d_kp >= 0.0f) cl->pi_d.kp = (d_kp > CUR_KP_MAX) ? CUR_KP_MAX : d_kp;
    if (d_ki >= 0.0f) cl->pi_d.ki = (d_ki > CUR_KI_MAX) ? CUR_KI_MAX : d_ki;
    if (q_kp >= 0.0f) cl->pi_q.kp = (q_kp > CUR_KP_MAX) ? CUR_KP_MAX : q_kp;
    if (q_ki >= 0.0f) cl->pi_q.ki = (q_ki > CUR_KI_MAX) ? CUR_KI_MAX : q_ki;
}


void CurrentLoop_Reset(CurrentLoop_t *cl)
{
    cl->pi_d.integral = 0;
    cl->pi_q.integral = 0;
    cl->vd = 0;
    cl->vq = 0;
}