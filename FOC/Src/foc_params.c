/* ===================================================================
 * foc_params.c —— 所有"设置接口"
 *
 * 这里不放任何控制逻辑, 只有: 改参数 / 读参数 / 切换模式。
 * 控制逻辑在 foc.c, 参数默认值在 foc_tune.h。
 *
 * 串口命令 → 本文件的哪个函数 (见 BSP/Src/bsp_cmd.c):
 *   mode / spd / pos ...   → FOC_SetMode 及各类 FOC_Set*
 *   clkp clki              → FOC_SetCurrentGains
 *   bw rs ls flux          → FOC_SetCurrentBandwidth / FOC_SetMotorParams
 *   spdkp spdki            → FOC_SetSpeedGains
 *   iql                    → FOC_SetSpeedLimit
 *   poskp poski poskd      → FOC_SetPositionGains / FOC_SetPositionDamping
 *   posv posacc posjerk    → FOC_SetPositionVmax / Acc / Jerk
 *   param                  → FOC_GetTune
 * =================================================================== */

#include "foc.h"
#include "foc_tune.h"
#include "foc_internal.h"
#include "bsp_led.h"
#include <math.h>

/* ===================================================================
 * 模式 / 故障
 * =================================================================== */

void FOC_SetMode(FOC_Mode_t mode)
{
    g_foc_mode = mode;
    align_cnt  = 0;

    /* 三个环的积分/状态全部清零 —— 切模式绝不能带着上一次的残余值 */
    CurrentLoop_Reset(&g_cl);
    SpeedLoop_Reset(&g_sl);
    PositionLoop_Reset(&g_pl);

    g_spd_ramp = 0.0f;

    g_fault_code = FOC_FAULT_NONE;
    BSP_LED_FaultOff();   /* 重新选模式 = 清故障指示 */
}

FOC_Mode_t FOC_GetMode(void)
{
    return g_foc_mode;
}

uint8_t FOC_GetFaultCode(void)
{
    return g_fault_code;
}

/* ===================================================================
 * 一次读出全部整定参数 (供 `param` 命令打印)
 * 注意: 读到的是【当前实时值】, 不是 foc_tune.h 里的默认值。
 * =================================================================== */
void FOC_GetTune(FOC_Tune_t *t)
{
    t->mot_rs  = g_motor_rs;    t->mot_ls   = g_motor_ls;    t->mot_flux  = g_motor_flux;
    t->cur_bw  = g_cur_bw;      t->cur_kp   = g_cl.pi_d.kp;  t->cur_ki    = g_cl.pi_d.ki;
    t->ff      = g_ff_gain;     t->vbus     = g_vbus;        t->dtc       = g_dt_comp;
    t->spd_kp  = g_sl.pi.kp;    t->spd_ki   = g_sl.pi.ki;    t->spd_iql   = g_sl.pi.out_max;
    t->spd_lpf = g_spd_lpf_ms;  t->spd_win_ms = (uint32_t)FOC_TUNE_SPD_WIN_MS;
    t->pos_kp  = g_pl.kp;       t->pos_ki   = g_pl.ki;       t->pos_kd    = g_pl.kd;
    t->pos_vmax= g_pl.v_max;    t->pos_acc  = g_pl.acc_max;  t->pos_jerk  = g_pl.jerk_max;
}

/* ===================================================================
 * 诊断读数 (调试打印用)
 * =================================================================== */

float FOC_GetCurRot(void)
{
    /* 探针1 (u0) 的方向角 = 电流测量在 theta=0 时看到的坐标旋转 (rad) */
    return atan2f(cal_u0b, cal_u0a);
}

float FOC_GetPosSpeedRef(void)
{
    /* 位置环当前输出的速度指令 (rad/s) */
    return g_pl.speed_ref;
}

float FOC_GetPosVraw(void)
{
    return g_pl.v_raw;      /* 限幅前的位置环输出 */
}

float FOC_GetPosVlim(void)
{
    return g_pl.v_lim;      /* 速度指令限幅 */
}

/* ===================================================================
 * PI 增益在线调节
 * =================================================================== */

void FOC_SetCurrentGains(float kp, float ki)
{
    CurrentLoop_SetGains(&g_cl, kp, ki, kp, ki);
}

void FOC_SetSpeedGains(float kp, float ki)
{
    SpeedLoop_SetGains(&g_sl, kp, ki);
}

void FOC_SetSpeedLimit(float amps)
{
    if (amps <= 0.0f)  return;
    if (amps > 10.0f)  amps = 10.0f;   /* 安全上限: 电机热额定自己拿握 */

    SpeedLoop_SetLimit(&g_sl, amps);
}

void FOC_SetPositionGains(float kp, float ki)
{
    /* 注意: PositionLoop_SetGains 参数顺序是 (ki, kp) */
    PositionLoop_SetGains(&g_pl, ki, kp);
}

void FOC_SetPositionDamping(float kd)
{
    PositionLoop_SetKd(&g_pl, kd);
}

void FOC_SetPositionVmax(float v_max)
{
    PositionLoop_SetVmax(&g_pl, v_max);
}

void FOC_SetPositionAcc(float acc_max)
{
    PositionLoop_SetAcc(&g_pl, acc_max);
}

void FOC_SetPositionJerk(float jerk_max)
{
    PositionLoop_SetJerk(&g_pl, jerk_max);
}

float FOC_GetPositionVref(void)
{
    return g_pl.v_max;
}

void FOC_PositionReset(void)
{
    PositionLoop_Reset(&g_pl);
}

/* ===================================================================
 * 电机参数 / 前馈 / 母线
 * =================================================================== */

void FOC_SetMotorParams(float rs, float ls, float flux)
{
    if (rs > 0.0f)    g_motor_rs   = rs;
    if (ls > 0.0f)    g_motor_ls   = ls;
    if (flux >= 0.0f) g_motor_flux = flux;
    FOC_ApplyCurrentGains();   /* R/L 一变, 电流环增益立即重算 */
}

void FOC_SetCurrentBandwidth(float bw)
{
    if (bw > 0.0f) g_cur_bw = bw;
    FOC_ApplyCurrentGains();
}

void FOC_SetFeedforward(float gain)
{
    if (gain < 0.0f) gain = 0.0f;
    if (gain > 1.0f) gain = 1.0f;
    g_ff_gain = gain;
}

void FOC_SetVbus(float v)
{
    if (v > 1.0f) g_vbus = v;
}

void FOC_SetDeadtimeComp(float duty)
{
    if (duty < 0.0f) duty = 0.0f;
    g_dt_comp = duty;
}
