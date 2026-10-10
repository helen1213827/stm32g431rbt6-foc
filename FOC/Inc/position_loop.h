#ifndef __POSITION_LOOP_H
#define __POSITION_LOOP_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"

/* ===================================================================
 * 位置环 —— 标准级联结构 (工程通用做法)
 *
 *   θ_cmd ─►[参考轨迹发生器]─► θ_ref , ω_ff
 *                                  │
 *                                  ▼
 *   θ ──►(−)──► Kp ──►(+)──►(±ω_max)──► ω_ref ──►[速度环 PI]──► iq
 *                       ▲
 *                       └── ω_ff
 *
 * 四条设计原则 (每一条都是为了干掉一个"特例补丁"):
 *
 * 1) 位置环只做 P。
 *    位置环里加积分 = 对位置做二次积分, 是"被外力顶住 → 蓄力 →
 *    松手弹射"的唯一来源。静差交给速度环/电流环的积分去消。
 *    这里只保留一个【严格限幅】的小积分 (±POS_INT_LIMIT, 不到 ω_max
 *    的 10%), 用来压齿槽静差 —— 它没有能力储能。
 *
 * 2) 唯一限幅 = 物理限幅 ω_max。
 *    没有刹车曲线、没有轮廓抗饱和、没有失速检测、没有静止消抖。
 *    上一版把这些塞进同一个函数里互相打架, 反而把环路激励成振荡。
 *    到位减速不是"算"出来的, 是 P 环的一阶收敛自然给的:
 *    ω_ref = Kp·e, 误差小则速度小。
 *
 * 3) 目标平滑与抗扰彻底解耦。
 *    下发新位置时的柔起柔停由【参考轨迹发生器】负责 —— 它只改 θ_ref,
 *    不进环路。手推电机的抗扰响应完全由 P 环 + 内层速度环决定,
 *    不经过轨迹发生器。所以"起步丝滑"和"被推后回位有力"不再互相
 *    牵制 (这正是上一版做不到的)。
 *
 * 4) 速度前馈 ω_ff。
 *    跟踪斜坡时把跟随误差压到 ~0, 不用靠大 Kp 去"追"误差;
 *    Kp 因此可以取小, 环路更稳。
 * =================================================================== */

typedef struct
{
    /* ---- 位置环 (纯 P) ---- */
    float kp;           /* (rad/s) 每 count 位置误差 */
    float ki;           /* 只用于压齿槽静差, 建议 0 */
    float kd;           /* 速度阻尼: 内层已是闭环速度环, 通常 0 */
    float int_term;     /* 严格限幅的积分项 (rad/s) */
    float speed_ref;    /* 输出: 速度指令 ω_ref (rad/s) */
    float v_max;        /* ω_ref 限幅 = 物理限幅 (rad/s) */

    /* ---- 参考轨迹发生器 (只对"指令变化"负责) ---- */
    float pos_ref;      /* 轨迹当前位置 (counts) */
    float v_traj;       /* 轨迹速度 (counts/s) */
    float a_traj;       /* 轨迹加速度 (counts/s²) */
    float v_max_c;      /* counts/s 版速度上限 */
    float acc_max_c;    /* counts/s² 版加速度上限 */
    float jerk_max_c;   /* counts/s³ 版加加速度上限 */

    /* ---- 调参用的物理单位副本 ---- */
    float acc_max;      /* 加速度上限 (rad/s²) */
    float jerk_max;     /* 加加速度上限 (rad/s³) */

    /* ---- 诊断 ---- */
    float v_raw;        /* 限幅前的位置环输出 */
    float e_traj;       /* 轨迹与指令之差 (counts) */
    float v_lim;        /* = v_max (兼容旧的调试打印) */

    bool  primed;       /* 首拍把 θ_ref 对齐到实际位置 */
    bool  enabled;
} PositionLoop_t;


void PositionLoop_Init(PositionLoop_t *pl, float kp, float ki, float kd, float v_max);
void PositionLoop_Run(PositionLoop_t *pl, float pos_cmd, uint16_t enc_raw, float speed, float dt);
void PositionLoop_SetGains (PositionLoop_t *pl, float ki, float kp);
void PositionLoop_SetKd (PositionLoop_t *pl, float kd);
void PositionLoop_SetVmax (PositionLoop_t *pl, float v_max);
void PositionLoop_SetAcc (PositionLoop_t *pl, float acc_max);
void PositionLoop_SetJerk(PositionLoop_t *pl, float jerk_max);
void PositionLoop_Reset(PositionLoop_t *pl);


#endif