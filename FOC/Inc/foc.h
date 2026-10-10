#ifndef FOC_H
#define FOC_H


#include "current_loop.h"
#include "speed_loop.h"
#include "position_loop.h"


#ifdef __cplusplus
extern "C" {
#endif


/* ---- 控制频率: 单一来源 ----
 * 必须与 CubeMX 里 TIM6 的实际频率一致!
 * 建议阶梯: 电流环 10kHz / 速度环 1kHz / 位置环 100Hz
 * foc.c 里所有周期、分频、时间窗都由它派生 —— 改控制频率只动这一个数。
 * 修改方法: TIM6 = 170MHz/(Prescaler+1)/(Period+1); 保持 Prescaler=169 (计数 1MHz)
 *           10kHz → Period=99   8kHz → 124   5kHz → 199   2kHz → 499 */
#define FOC_FREQ_HZ     (10000U)
#define FOC_DT          (1.0f / (float)FOC_FREQ_HZ)

#define VBUS_NOMINAL    (12.0f)   /* 母线名义电压 (V) */
#define VBUS_HALF       (VBUS_NOMINAL / 2.0f)
#define VBUS_LINEAR_MAX (VBUS_NOMINAL / 1.7320508f)  /* SVPWM 线性区上限: Vbus/√3 */


typedef enum
{
    FOC_MODE_DISABLED = 0,
    FOC_MODE_ALIGN,
    FOC_MODE_VOLATAGE_OPENLOOP,
    FOC_MODE_CURRENT,
    FOC_MODE_SPEED,
    FOC_MODE_POSITION,
} FOC_Mode_t;

extern FOC_Mode_t g_foc_mode;
extern float g_vd_open;
extern float g_vq_open;
extern float g_id_ref;
extern float g_iq_ref;
extern float g_off;          /* 校准后的电零点偏移(rad), debugger 可实时微调 */
extern float g_align_vd;     /* 开环对齐时的 d 轴电压 (V) */
extern uint16_t g_enc_cache; /* FOC 循环缓存的编码器原始值 (供打印, 避免 SPI 冲突) */

/* 电流测量坐标补偿矩阵 (静止系, true = C·measured)
 * ALIGN 双探针自动标定, 修正相位映射不一致(旋转/镜像/增益差) */
typedef struct {
    float c00, c01;
    float c10, c11;
} CurFrame_t;
extern CurFrame_t g_cur_frame;
extern float g_iq_sign;      /* q 轴电流反馈符号 (±1), debugger 可调 */
extern float g_spd_sign;     /* 速度环输出符号 (±1), debugger 可调 */

/* 标定出的坐标旋转角 (供打印/观测, 单位 rad) */
float FOC_GetCurRot(void);
float FOC_GetPosSpeedRef(void);
float FOC_GetPosVraw(void);   /* 位置环限幅前的 PI 原始输出 (诊断) */
float FOC_GetPosVlim(void);   /* 位置环制动约束上限 (诊断) */

/* PI 增益在线调节 (串口命令用) */
void FOC_SetCurrentGains(float kp, float ki);
void FOC_SetSpeedGains(float kp, float ki);
void FOC_SetSpeedLimit(float amps);      /* 速度环输出限幅 = 最大 iq (A) */
void FOC_SetPositionGains(float kp, float ki);
void FOC_SetPositionDamping(float kd);
void FOC_SetPositionVmax(float v_max);   /* 位置环最大速度指令 (rad/s) */
void FOC_SetPositionAcc(float acc_max);  /* 位置环速度指令加速度限幅 (rad/s²) */
void FOC_SetPositionJerk(float jerk_max);/* 参考轨迹加加速度上限 (rad/s³), 越小越丝滑 */
uint8_t FOC_GetFaultCode(void);          /* 保护停机原因: 0=无 1=KEY 2=失控 3=失速 */

/* 一次读出全部整定参数 (供 `param` 命令打印)。
 * 默认值都在 FOC/Inc/foc_tune.h, 这里读出的是【当前实时值】。 */
typedef struct
{
    float mot_rs, mot_ls, mot_flux;                     /* 电机 */
    float cur_bw, cur_kp, cur_ki;                       /* 电流环 */
    float ff, vbus, dtc;                                /* 前馈/母线/死区 */
    float spd_kp, spd_ki, spd_iql, spd_lpf;             /* 速度环 */
    uint32_t spd_win_ms;                                /* 测速滑窗 (编译期) */
    float pos_kp, pos_ki, pos_kd;                       /* 位置环 */
    float pos_vmax, pos_acc, pos_jerk;                  /* 位置参考轨迹 */
} FOC_Tune_t;
void FOC_GetTune(FOC_Tune_t *t);
float FOC_GetPositionVref(void);
void FOC_PositionReset(void);

extern float g_speed_ref;
extern float g_spd_ramp;     /* 速度参考斜坡当前值 (斜坡后的真实参考, 供 dbg 3 打印) */
extern uint8_t g_stall_enable;  /* 失速保护开关 (默认 0=关) */
extern float   g_spd_lpf_ms;     /* 测速低通时间常数 (ms), spdf 命令可改 */
extern float g_pos_ref;
extern float g_iq_ref_cmd;
extern float g_sl_iq_ref_out;

/* 电流低通滤波值: 只用于打印/观测 (闭环内环一律用瞬时 motor.id/motor.iq) */
extern float g_id_filt;
extern float g_iq_filt;
extern float g_spd_min, g_spd_max;   /* 测速极值 (dbg 2) */
void FOC_SpdRangeReset(void);       /* 把极值重置为当前速度 */
/* ===================================================================
 * 电机电气参数 / 电流环整定 (全部可串口在线修改)
 * ===================================================================
 * 电流环按零极点对消整定: Kp = ωc·L, Ki = ωc·R
 * Kp 与 Ki 必须来自同一个 ωc, 否则积分不起作用或环路振荡。 */
extern float g_motor_rs;     /* 相电阻 R (Ω) */
extern float g_motor_ls;     /* 相电感 L (H) */
extern float g_motor_flux;   /* 永磁磁链 ψf (Wb), 0 = 不补偿反电动势 */
extern float g_cur_bw;       /* 电流环目标带宽 ωc (rad/s) */
extern float g_ff_gain;      /* dq 解耦+反电动势前馈强度 0~1 (0=关) */
extern float g_vbus;         /* 母线电压 (V) */
extern float g_dt_comp;      /* 死区补偿占空比 (0=关, 约 0.047) */
extern float g_omega_e;      /* 估计的电角速度 (rad/s), 解耦用 */

void FOC_SetMotorParams(float rs, float ls, float flux);
void FOC_SetCurrentBandwidth(float bw);
void FOC_SetFeedforward(float gain);
void FOC_SetVbus(float v);
void FOC_SetDeadtimeComp(float duty);

/* 串口调试打印分组 (dbg 命令切换)
 * 0=关 1=电流环 2=测速 3=速度环 4=位置环 5=全量 */
extern uint8_t g_dbg_mode;

/* 未补偿的静止系电流 (ALIGN 双探针用; dbg 6 可观测原始值) */
extern float ia_raw_filt;
extern float ib_raw_filt;

/* 打印窗口内的电流极值 (用于发现被 100ms 采样掩盖的振荡) */
extern float g_id_min, g_id_max;
extern float g_iq_min, g_iq_max;
void FOC_CurRangeReset(void);

/* ISR 占用率 (us) — Cortex-M4 DWT 周期计数器测得 */
uint32_t FOC_IsrLoadAvgUs(void);
uint32_t FOC_IsrLoadMaxUs(void);
uint32_t FOC_IsrLoadLastUs(void);
void     FOC_IsrLoadReset(void);

void FOC_Init(void);

void FOC_Run(void);

void FOC_SetMode(FOC_Mode_t mode);


FOC_Mode_t FOC_GetMode(void);

#ifdef __cplusplus
}
#endif

#endif