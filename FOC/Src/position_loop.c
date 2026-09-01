#include "position_loop.h"
#include "pid.h"

/* 在线调参安全限幅 */
#define POS_KP_MAX  1.0f
#define POS_KI_MAX  0.1f
#define POS_KD_MAX  2.0f

/* 位置环积分抗饱和 */
#define POS_INT_LIMIT  1.0f     /* 积分上限 (rad/s 当量), 防长距离接近时积分涨满顶住 P 项 */
#define POS_INT_BAND   1000.0f  /* 条件积分带: 距目标 >1000 counts 时不积分 */

/* 平滑: 速度指令加速度限幅 (rad/s²) — 只在行程区生效, 梯形速度曲线 */
#define POS_ACC_MAX     10.0f
/* 接近目标多少 counts 内直通(不做限幅), 避免限速滞后导致摆头 */
#define POS_SMOOTH_BAND 3000.0f

void PositionLoop_Init(PositionLoop_t *pl, float kp, float ki, float kd, float out_limit)
{
    PI_Init(&pl->pi, kp, ki, -out_limit, out_limit);
    pl->pi.int_limit = POS_INT_LIMIT;
    pl->speed_ref = 0;
    pl->kd = kd;
    pl->sm_cmd = 0;
    pl->enabled = true;
}

void PositionLoop_Run(PositionLoop_t *pl, float pos_ref, uint16_t enc_raw, float speed, float dt)
{
    if (!pl->enabled) { pl->speed_ref = 0.0f; return;}

    /* 环绕修正: 取"最近路径"的误差 (±32768 内), 跨过编码器 0/65535 边界也不会跑错方向 */
    float pos_err = pos_ref - (float)enc_raw;
    if      (pos_err >  32768.0f) pos_err -= 65536.0f;
    else if (pos_err < -32768.0f) pos_err += 65536.0f;

    /* 条件积分抗饱和: 远离目标时清零积分, 只让积分在靠近目标时累积
     * (否则长距离接近时积分涨满, 到目标后被积分顶住/推过头) */
    if (pos_err > POS_INT_BAND || pos_err < -POS_INT_BAND)
        pl->pi.integral = 0.0f;

    /* PI 项 (Kp*err + Ki*∫err), 用修正后的误差驱动:
     * PI_Run 内部 error = ref - fb, 所以令 ref = enc + pos_err → error = pos_err
     * 再减速度阻尼项 Kd*speed: 电机越快刹车越狠, 接近目标自动提前减速, 防冲过头 */
    float cmd = PI_Run(&pl->pi, (float)enc_raw + pos_err, (float)enc_raw, dt) - pl->kd * speed;

    /* 平滑: 只在远离目标(行程移动)时做加速度限幅 → 起步/减速平滑;
     * 接近目标(±POS_SMOOTH_BAND 内)直通, 避免限速滞后导致摆头 */
    if (pos_err > POS_SMOOTH_BAND || pos_err < -POS_SMOOTH_BAND)
    {
        float step = POS_ACC_MAX * dt;
        if      (cmd - pl->sm_cmd >  step) pl->sm_cmd += step;
        else if (cmd - pl->sm_cmd < -step) pl->sm_cmd -= step;
        else                               pl->sm_cmd  = cmd;
    }
    else
    {
        pl->sm_cmd = cmd;   /* 接近目标直通, 无滞后 */
    }

    pl->speed_ref = pl->sm_cmd;
}

void PositionLoop_SetGains (PositionLoop_t *pl, float ki, float kp)
{
    /* -1 (或任何负值) 表示"这一项不改" */
    if (ki >= 0.0f) pl->pi.ki = (ki > POS_KI_MAX) ? POS_KI_MAX : ki;
    if (kp >= 0.0f) pl->pi.kp = (kp > POS_KP_MAX) ? POS_KP_MAX : kp;
}

void PositionLoop_SetKd (PositionLoop_t *pl, float kd)
{
    if (kd >= 0.0f) pl->kd = (kd > POS_KD_MAX) ? POS_KD_MAX : kd;
}

void PositionLoop_Reset(PositionLoop_t *pl)
{
    pl->speed_ref = 0;
    pl->sm_cmd = 0;
    pl->pi.integral = 0;
}