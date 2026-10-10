#include "position_loop.h"
#include "pid.h"
#include <math.h>

/* ===================================================================
 * 位置环 —— 标准级联结构。设计说明见 position_loop.h。
 *
 * 这个文件里【只有两件事】:
 *   1) 参考轨迹发生器  (把 θ_cmd 揉成 θ_ref / ω_ff)
 *   2) 纯 P 位置环      (ω_ref = ω_ff + Kp·e, 限幅到 ω_max)
 * 没有刹车曲线、没有抗饱和钳位、没有失速检测。
 * =================================================================== */

/* ---- 在线调参安全限幅 ---- */
#define POS_KP_MAX      2.0f
#define POS_KI_MAX      0.5f
#define POS_KD_MAX      5.0f
#define POS_VMAX_MAX   40.0f
#define POS_ACC_MAX_LIM 2000.0f
#define POS_JERK_MAX   50000.0f

/* ---- 位置环 (纯 P) ---- */
/* 位置环积分的授权。
 * 0.3 rad/s 太小: 内环 kp 只有 0.01 (几乎纯积分), 0.3 rad/s 的速度指令
 * 过内环后只剩几十 mA —— 根本顶不住齿槽/静摩擦, 表现就是"有静差"、"太软"。
 * 0.8 rad/s 才够让积分真正去消静差。
 * 靠 POS_INT_BAND 大门: 误差 > 1000 counts 时积分直接清零,
 * 所以手推走 20000 counts 的过程中积分永远是 0 —— 不会因此产生松手反冲。 */
#define POS_INT_LIMIT   0.8f    /* rad/s */
#define POS_INT_BAND   1000.0f  /* counts: 只有接近目标才允许积分 */

/* 位置误差限幅: 纯粹是"手掰半圈"这种极端情况的保护,
 * 正常工作区(几百 counts)完全不受影响。 */
#define POS_ERR_CLAMP  30000.0f

/* counts ↔ 机械弧度 */
#define POS_CNT2RAD    (6.2831853f / 65536.0f)
#define POS_RAD2CNT    (65536.0f / 6.2831853f)

/* ---- 参考轨迹发生器 ---- */
/* 轨迹对指令误差的比例响应 (1/s): 时间常数 ~50ms, 远快于位置环带宽,
 * 不拖累指令跟踪; 真正决定运动形状的是 v / acc / jerk 三个限幅。 */
#define POS_TRAJ_GAIN   20.0f

/* 轨迹速度对目标速度的跟踪增益 (1/s): 期望加速度 = (v_des − v_traj)×此值 */
#define POS_A_GAIN      50.0f

/* 跟随误差上限 (counts): 超过就冻住轨迹 (工业伺服的 position error limit)。
 * 防止"被外力顶住时参考还在往前走", 松手变成大误差冲击。 */
#define POS_FERR_MAX    5000.0f

/* 默认轨迹加速度 (rad/s²)。
 * ⚠️ 必须 ≤ 机器实际最大加速度  a_max = Kt/J × iq_max。
 * 实测本机 Kt/J ≈ 10~25 rad/s²/A, 速度环 iql=2A → a_max ≈ 20~50 rad/s²。
 * 设成机器达不到的值 = 整段轨迹都在饱和 → 必然冲过头。 */
#define POS_ACC_DEF     15.0f

/* 默认加加速度上限 (rad/s³): 加速度的斜率。越小越丝滑、越大越干脆。
 * 300 对应"加速度用 50ms 从 0 升到 15rad/s²"。 */
#define POS_JERK_DEF   300.0f

void PositionLoop_Init(PositionLoop_t *pl, float kp, float ki, float kd, float v_max)
{
    pl->kp        = kp;
    pl->ki        = ki;
    pl->kd        = kd;
    pl->int_term  = 0.0f;
    pl->speed_ref = 0.0f;

    pl->v_max     = v_max;
    pl->v_max_c   = v_max * POS_RAD2CNT;
    pl->v_lim     = v_max;

    PositionLoop_SetAcc (pl, POS_ACC_DEF);
    PositionLoop_SetJerk(pl, POS_JERK_DEF);

    pl->pos_ref = 0.0f;
    pl->v_traj  = 0.0f;
    pl->a_traj  = 0.0f;
    pl->v_raw   = 0.0f;
    pl->e_traj  = 0.0f;
    pl->primed  = false;
    pl->enabled = true;
}

void PositionLoop_Run(PositionLoop_t *pl, float pos_cmd, uint16_t enc_raw, float speed, float dt)
{
    if (!pl->enabled) { pl->speed_ref = 0.0f; return; }

    float theta = (float)enc_raw;

    /* 首拍: θ_ref 对齐到实际位置, 避免上电/切模式时弹一下 */
    if (!pl->primed)
    {
        pl->pos_ref = theta;
        pl->v_traj  = 0.0f;
        pl->a_traj  = 0.0f;
        pl->primed  = true;
    }

    /* ================= 1) 参考轨迹发生器 =================
     * 只对【指令变化】负责。手推电机时 θ_cmd 不动, 轨迹早就在位了,
     * 剩下的全靠下面的 P 环 —— 目标平滑与抗扰在这里彻底解耦。 */

    /* θ_cmd 取离 θ_ref 最近的那一圈 */
    float e_cmd = pos_cmd - pl->pos_ref;
    if      (e_cmd >  32768.0f) e_cmd -= 65536.0f;
    else if (e_cmd < -32768.0f) e_cmd += 65536.0f;

    /* 参考与实际的跟随误差 */
    float e_fb = pl->pos_ref - theta;
    if      (e_fb >  32768.0f) e_fb -= 65536.0f;
    else if (e_fb < -32768.0f) e_fb += 65536.0f;

    pl->e_traj = e_cmd;

    /* 期望轨迹速度: 既要追指令误差, 又不能超出"从当前位置刹得住"的上限 */
    float v_des = e_cmd * POS_TRAJ_GAIN;
    float v_cap = sqrtf(2.0f * pl->acc_max_c * fabsf(e_cmd));
    if (v_cap > pl->v_max_c) v_cap = pl->v_max_c;
    if (v_des >  v_cap) v_des =  v_cap;
    if (v_des < -v_cap) v_des = -v_cap;

    if (fabsf(e_fb) < POS_FERR_MAX)
    {
        /* 加速度限幅 + 加加速度限幅 → S 形轮廓 */
        float a_des = (v_des - pl->v_traj) * POS_A_GAIN;
        if (a_des >  pl->acc_max_c) a_des =  pl->acc_max_c;
        if (a_des < -pl->acc_max_c) a_des = -pl->acc_max_c;

        float da     = a_des - pl->a_traj;
        float da_max = pl->jerk_max_c * dt;
        if (da >  da_max) da =  da_max;
        if (da < -da_max) da = -da_max;
        pl->a_traj += da;
    }
    else
    {
        /* 跟随误差超限 → 冻结轨迹 (参考不再前进),
         * 防止被外力顶住时参考一直跑、松手变成大误差冲击。 */
        pl->a_traj = 0.0f;
    }

    pl->v_traj += pl->a_traj * dt;
    if (pl->v_traj >  pl->v_max_c) pl->v_traj =  pl->v_max_c;
    if (pl->v_traj < -pl->v_max_c) pl->v_traj = -pl->v_max_c;

    pl->pos_ref += pl->v_traj * dt;

    /* ================= 2) 位置环: 纯 P + 速度前馈 =================
     * ω_ref = ω_ff + Kp·e (+ 小积分) − Kd·ω
     * 到位减速不是算出来的, 是 P 环的一阶收敛自然给的:
     * 误差小则速度指令小, 而且它没有积分可以蓄力。 */
    float e_pos = pl->pos_ref - theta;
    if      (e_pos >  32768.0f) e_pos -= 65536.0f;
    else if (e_pos < -32768.0f) e_pos += 65536.0f;

    if (e_pos >  POS_ERR_CLAMP) e_pos =  POS_ERR_CLAMP;
    if (e_pos < -POS_ERR_CLAMP) e_pos = -POS_ERR_CLAMP;

    /* 小积分: 只在接近目标时累积, 且严格限幅 → 只能消静差, 不能储能 */
    if (pl->ki > 0.0f && fabsf(e_pos) < POS_INT_BAND)
    {
        pl->int_term += e_pos * pl->ki * dt;
        if (pl->int_term >  POS_INT_LIMIT) pl->int_term =  POS_INT_LIMIT;
        if (pl->int_term < -POS_INT_LIMIT) pl->int_term = -POS_INT_LIMIT;
    }
    else
    {
        pl->int_term = 0.0f;
    }

    float w = pl->v_traj * POS_CNT2RAD      /* 速度前馈 ω_ff */
            + pl->kp * e_pos                /* 位置 P */
            + pl->int_term
            - pl->kd * speed;

    pl->v_raw = w;

    /* 唯一的限幅 = 物理限幅 ω_max */
    if (w >  pl->v_max) w =  pl->v_max;
    if (w < -pl->v_max) w = -pl->v_max;

    pl->speed_ref = w;
}

void PositionLoop_SetGains (PositionLoop_t *pl, float ki, float kp)
{
    /* -1 (或任何负值) 表示"这一项不改" */
    if (ki >= 0.0f) pl->ki = (ki > POS_KI_MAX) ? POS_KI_MAX : ki;
    if (kp >= 0.0f) pl->kp = (kp > POS_KP_MAX) ? POS_KP_MAX : kp;
}

void PositionLoop_SetKd (PositionLoop_t *pl, float kd)
{
    if (kd >= 0.0f) pl->kd = (kd > POS_KD_MAX) ? POS_KD_MAX : kd;
}

void PositionLoop_SetVmax (PositionLoop_t *pl, float v_max)
{
    if (v_max <= 0.0f) return;
    if (v_max > POS_VMAX_MAX) v_max = POS_VMAX_MAX;

    pl->v_max   = v_max;
    pl->v_max_c = v_max * POS_RAD2CNT;
    pl->v_lim   = v_max;
}

void PositionLoop_SetAcc (PositionLoop_t *pl, float acc_max)
{
    if (acc_max <= 0.0f) return;
    if (acc_max > POS_ACC_MAX_LIM) acc_max = POS_ACC_MAX_LIM;

    pl->acc_max   = acc_max;
    pl->acc_max_c = acc_max * POS_RAD2CNT;
}

void PositionLoop_SetJerk (PositionLoop_t *pl, float jerk_max)
{
    if (jerk_max <= 0.0f) return;
    if (jerk_max > POS_JERK_MAX) jerk_max = POS_JERK_MAX;

    pl->jerk_max   = jerk_max;
    pl->jerk_max_c = jerk_max * POS_RAD2CNT;
}

void PositionLoop_Reset(PositionLoop_t *pl)
{
    pl->pos_ref   = 0.0f;
    pl->v_traj    = 0.0f;
    pl->a_traj    = 0.0f;
    pl->speed_ref = 0.0f;
    pl->int_term  = 0.0f;
    pl->primed    = false;   /* 下次运行用实测位置重新对齐参考 */
}