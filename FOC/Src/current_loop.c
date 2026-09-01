#include "current_loop.h"
#include "pid.h"

/* 在线调参安全限幅: 防止误设巨大/负增益导致跑飞 */
#define CUR_KP_MAX  20.0f
#define CUR_KI_MAX  200.0f

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
float id_fb, float iq_fb, float dt)
{
    if (!cl->enabled)  { cl->vd = 0.0f; cl->vq = 0.0f; return;}

    cl->vd = PI_Run(&cl->pi_d, id_ref, id_fb, dt);
    cl->vq = PI_Run(&cl->pi_q, iq_ref, iq_fb, dt);


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