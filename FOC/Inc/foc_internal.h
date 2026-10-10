#ifndef __FOC_INTERNAL_H
#define __FOC_INTERNAL_H

/* ===================================================================
 * FOC 模块内部的共享变量声明
 *
 * ⚠️ 这个头文件是【模块内部】用的, 外部代码不应该 include 它。
 *    外部只 include foc.h 和 foc_tune.h。
 *
 * 为什么要它: 以前 foc.c 把所有东西塞在一个文件里, 所以变量都是 static,
 * 没有这个头也能编。现在拆成 foc.c(控制逻辑) + foc_params.c(设置接口),
 * 两个文件都要用到三个环的对象, 就在这里统一声明。
 * =================================================================== */

#include <stdint.h>
#include "foc.h"
#include "current_loop.h"
#include "speed_loop.h"
#include "position_loop.h"

/* ---------------- 三个环的对象 ---------------- */
extern CurrentLoop_t  g_cl;     /* 电流环 (最快, 10kHz) */
extern SpeedLoop_t    g_sl;     /* 速度环 (1kHz) */
extern PositionLoop_t g_pl;     /* 位置环 (100Hz, 最外) */

/* ---------------- 运行模式 / 故障 ---------------- */
extern volatile uint8_t g_fault_code;

/* 保护停机的原因码。定义在这里(而不是 foc.c)是因为 foc_params.c
 * 的 FOC_SetMode 要清它。 */
#define FOC_FAULT_NONE      0U
#define FOC_FAULT_KEY       1U   /* KEY 引脚被拉低 */
#define FOC_FAULT_OVERSPEED 2U   /* 失控: 转速远超指令 */
#define FOC_FAULT_STALL     3U   /* 失速: 速度模式下堵转 */

/* ---------------- 电机参数 / 整定 ---------------- */
extern float g_motor_rs;        /* 相电阻 (Ω) */
extern float g_motor_ls;        /* 相电感 (H) */
extern float g_motor_flux;      /* 磁链 (Wb) */
extern float g_cur_bw;          /* 电流环带宽 ωc (rad/s) */
extern float g_ff_gain;         /* 前馈强度 0~1 */
extern float g_vbus;            /* 母线电压 (V) */
extern float g_dt_comp;         /* 死区补偿占空比 */

/* ---------------- 测速 ---------------- */
extern float g_spd_lpf_ms;      /* 测速低通时间常数 (ms) */
extern float g_spd_min;         /* 打印窗口内速度极小值 */
extern float g_spd_max;         /* 打印窗口内速度极大值 */

/* ---------------- 校准状态 ---------------- */
extern uint32_t align_cnt;      /* ALIGN 已走过的拍数 */
extern float cal_u0a, cal_u0b;  /* 探针1 测到的电流方向 */

/* ---------------- 速度参考斜坡状态 ---------------- */
extern float g_spd_ramp;

/* ---------------- 模块内部函数 (foc.c 提供) ---------------- */
void FOC_ApplyCurrentGains(void);   /* 由 R/L/ωc 重算电流环 Kp/Ki */

#endif
