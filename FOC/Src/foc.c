#include "foc.h"
#include "bsp_encoder.h"
#include "clarke.h"
#include "current_loop.h"
#include "math_types.h"
#include "park.h"
#include "motor.h"
#include "current.h"
#include "inv_park.h"
#include "position_loop.h"
#include "speed_loop.h"
#include "stm32g4xx.h"
#include "svpwm.h"
#include "bsp_pwm.h"
#include "angle.h"
#include "bsp_led.h"
#include <math.h>
#include <stdint.h>


#define FOC_DT  (0.001f)
#define VBUS    (12.0f)
#define VBUS_HALF  (VBUS / 2.0f)
#define CUR_DT  (0.001f)
#define SPD_DT  (0.01f)
#define POS_DT  (0.01f)
#define SPD_DIV (10U)
#define POS_DIV (10U)
#define SPD_RAMP_RATE (4.0f)   /* 速度参考斜坡: 4 rad/s², 避免阶跃冲击 */

static CurrentLoop_t g_cl;
static SpeedLoop_t   g_sl;
static PositionLoop_t g_pl;

static AlphaBeta_t i_ab,v_ab;
static DQ_t         i_dq;
static SVPWM_t      pwm;

FOC_Mode_t g_foc_mode = FOC_MODE_ALIGN;  /* 上电先对齐(双探针校准), 不能跳过 */
static bool g_calibrated = false;        /* ALIGN 校准是否完成 */
float g_vd_open     = 0.0f;
float g_vq_open     = 3.0f;
float g_id_ref      = 0.0f;   /* 校准: 定位用 d 轴电流 0.5A */
float g_iq_ref      = -0.15f;   /* 校准: q 轴先给 0 */
float g_off         = 0.0f;   // 校准后的电零点偏移, debugger 可改
float g_align_vd    = 6.0f;   // 开环对齐电压 (12V 母线, 6V 安全)
CurFrame_t g_cur_frame = { 1.0f, 0.0f, 0.0f, 1.0f };  /* 默认单位阵 */
float g_iq_sign      = 1.0f;  /* q 轴电流反馈符号 (d轴正确,q轴反了 → 翻转) */
float g_spd_sign     = -1.0f; /* 速度环输出符号 (iq_ref<0→+mech, 需取反) */
uint16_t g_enc_cache = 0;     // FOC 循环缓存编码器值, 供主循环打印
float g_speed_ref   = 10.0f;   /* 速度环目标 (rad/s, 机械角速度) */
float g_pos_ref     = 500.0f;

static uint32_t align_cnt = 0;
#define ALIGN_TIME 3000

/* 电流低通滤波 (各模式共用) */
static float id_filt = 0.0f, iq_filt = 0.0f;

/* 电流指令 (iq_ref) 打印用, 诊断速度环 vs 电流环 */
float g_iq_ref_cmd = 0.0f;
float g_sl_iq_ref_out = 0.0f;  /* 速度环原始输出 */
/* 未补偿的静止系电流滤波, 供 ALIGN 双探针标定 */
static float ia_raw_filt = 0.0f, ib_raw_filt = 0.0f;
/* 探针1 (vd) 标定的 u0 */
static float cal_u0a = 1.0f, cal_u0b = 0.0f;
/* 速度参考斜坡状态 */
static float spd_ramp = 0.0f;

static void FOC_ReadCurrents(void)
{
    motor.ia = current.ia;
    motor.ib = current.ib;
    motor.ic = current.ic;

    ABC_t abc = {.a = motor.ia, .b = motor.ib, .c = motor.ic};
    Clarke_Run(&abc, &i_ab);

    float alpha = 0.1f;                    /* 1kHz 下 α=0.1 → τ≈10ms */

    /* 原始(未补偿)静止系电流, 供双探针标定坐标矩阵 */
    ia_raw_filt = ia_raw_filt * (1.0f - alpha) + i_ab.alpha * alpha;
    ib_raw_filt = ib_raw_filt * (1.0f - alpha) + i_ab.beta  * alpha;

    /* 电流测量坐标补偿 (静止系, Park 之前): true = C·measured
     * 修正电流采样与驱动相位映射不一致(旋转/镜像/增益差) */
    float ra = i_ab.alpha, rb = i_ab.beta;
    i_ab.alpha = g_cur_frame.c00 * ra + g_cur_frame.c01 * rb;
    i_ab.beta  = g_cur_frame.c10 * ra + g_cur_frame.c11 * rb;

    Park_Run(&i_ab, motor.theta, &i_dq);
    motor.id = i_dq.d;
    motor.iq = g_iq_sign * i_dq.q;   /* q 轴符号修正: 使 q 环为负反馈 */

    id_filt = id_filt * (1.0f - alpha) + motor.id * alpha;
    iq_filt = iq_filt * (1.0f - alpha) + motor.iq * alpha;
}



void FOC_Init(void)
{
    /* 电流环: 已验证参数 Kp=0.5 Ki=5.0, 输出限幅 ±VBUS_HALF(±6V) */
    CurrentLoop_Init(&g_cl, 0.5f, 5.0f, VBUS_HALF);
    /* 速度环: 环路修复后(输出真正生效)先从保守值起步, 再往上调
     * Kp=0.2, 输出限幅 ±2A, 速度滤波 α=0.4 */
    SpeedLoop_Init(&g_sl, 0.2f, 0.03f, 2.0f, 0.4f);
    /* 位置环: 扭矩(刚度)与最大速度解耦
     * kp=0.004 (掰一下扭矩更大), 速度限幅仍 ±2(→不转圈平滑)
     * kd=1.5 (随kp加阻尼), ki=0.0005 顶齿槽 */
    PositionLoop_Init(&g_pl, 0.004f, 0.0005f, 1.5f, 2.0f);
    Angle_Init();
    Angle_SetMode(ANGLE_MODE_ENCODER);
    Current_Init();
}


void FOC_Run(void)
{
        if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
    {
        g_foc_mode = FOC_MODE_DISABLED;
    }
    Current_Update();

    /* 保护: 未完成 ALIGN 校准前, 不允许进入电流/速度/位置模式
     * (跳过校准会导致坐标错误、电流环推不动机器、过热) */
    if (g_foc_mode >= FOC_MODE_CURRENT && !g_calibrated)
    {
        g_foc_mode = FOC_MODE_ALIGN;
    }

    /* 编码器只在这里读一次 (1kHz, ISR 上下文), 主循环打印用缓存 → 避免 SPI 冲突 */
    g_enc_cache = BSP_Encoder_ReadAngle();
    float mech  = BSP_Encoder_CountToRad(g_enc_cache);

    switch(g_foc_mode)
    {
        case FOC_MODE_DISABLED:
            /* 不上电; 仅跟踪角度, 供手动旋转验证方向 (阶段2) */
            Angle_Update(mech, CUR_DT);
            motor.theta = Angle_GetTheta();
            motor.vd = 0.0f;
            motor.vq = 0.0f;
            break;

        case FOC_MODE_ALIGN:
        {
            /* 双探针坐标标定 (theta=0 固定):
             *   前半段: vd=6, vq=0 → 真实电流沿 +α, 测得 u0
             *   后半段: vd=0, vq=6 → 真实电流沿 +β, 测得 u1
             * 由 u0/u1 构建静止系补偿矩阵 C = E^T, 修正旋转/镜像/增益差 */
            motor.theta = 0.0f;
            FOC_ReadCurrents();

            if (align_cnt < ALIGN_TIME/2)
            {
                motor.vd = g_align_vd;
                motor.vq = 0.0f;

                if (align_cnt == ALIGN_TIME/2 - 1)   /* probe1 尾段, 转子已静止 */
                {
                    float mag = sqrtf(ia_raw_filt*ia_raw_filt + ib_raw_filt*ib_raw_filt);
                    if (mag > 0.3f)
                    {
                        cal_u0a = ia_raw_filt / mag;
                        cal_u0b = ib_raw_filt / mag;
                    }
                    /* 电零点偏移在 probe1 位置标定!
                     * (probe1: 转子对齐 +α = 控制器 d 轴; probe2 会转到 q 轴) */
                    g_off = Angle_CalibrateOffset();
                }
            }
            else
            {
                motor.vd = 0.0f;
                motor.vq = g_align_vd;
            }

            align_cnt++;
            if(align_cnt >= ALIGN_TIME)
            {
                /* probe2 尾段: 测 u1, 构建补偿矩阵 C = E^{-1} (E=[u0|u1])
                 * 用真逆而非转置: 坐标误差含增益不一致时 E 不正交, E^T≠E^{-1} */
                float mag = sqrtf(ia_raw_filt*ia_raw_filt + ib_raw_filt*ib_raw_filt);
                if (mag > 0.3f)
                {
                    float u1a = ia_raw_filt / mag;
                    float u1b = ib_raw_filt / mag;

                    float det = cal_u0a * u1b - u1a * cal_u0b;
                    if (fabsf(det) > 0.1f)
                    {
                        g_cur_frame.c00 =  u1b      / det;
                        g_cur_frame.c01 = -u1a      / det;
                        g_cur_frame.c10 = -cal_u0b  / det;
                        g_cur_frame.c11 =  cal_u0a  / det;
                    }
                }

                /* g_off 已在 probe1 位置标定, 这里不再重算 (probe2 转子在 q 轴) */
                align_cnt = 0;
                g_calibrated = true;   /* 校准完成, 允许进入电流/速度/位置模式 */
                FOC_SetMode(FOC_MODE_SPEED);   /* 上电校准后自动进速度环, 不用手打 mode 4 */
            }
        }
        break;

        case FOC_MODE_VOLATAGE_OPENLOOP:

                Angle_Update(mech, CUR_DT);
                motor.theta = Angle_GetTheta();

                /* 读电流并变换, 只看不控 */
                FOC_ReadCurrents();

                motor.vd = g_vd_open;
                motor.vq = g_vq_open;
                break;
        
        case FOC_MODE_CURRENT:
        case FOC_MODE_SPEED:
        case FOC_MODE_POSITION:
        {
            /* 每次应用 g_off (debugger 可实时微调 offset) */
            Angle_SetOffset(g_off);
            Angle_Update(mech, CUR_DT);
            motor.theta = Angle_GetTheta();

            FOC_ReadCurrents();

            uint16_t enc = g_enc_cache;

            static uint32_t lo_cnt = 0;
            lo_cnt++;

            float iq_ref    = g_iq_ref;
            float speed_ref = g_speed_ref;

            if((lo_cnt % SPD_DIV) == 0)
            {
                if (g_foc_mode == FOC_MODE_POSITION && (lo_cnt % POS_DIV) == 0)
                {
                    PositionLoop_Run(&g_pl, g_pos_ref, enc, g_sl.speed, POS_DT);
                    speed_ref = g_pl.speed_ref;
                }

                if (g_foc_mode == FOC_MODE_SPEED)
                {
                    /* 速度参考斜坡: 仅速度模式用, 防阶跃超调 */
                    float diff = speed_ref - spd_ramp;
                    float step = SPD_RAMP_RATE * SPD_DT;
                    if      (diff >  step) spd_ramp += step;
                    else if (diff < -step) spd_ramp -= step;
                    else                   spd_ramp  = speed_ref;

                    SpeedLoop_Run(&g_sl, spd_ramp, (float)enc, SPD_DT);
                }
                else   /* POSITION: 位置环输出直连速度环, 绕过斜坡
                        * (斜坡减速太慢 4rad/s², 高速逼近目标刹不住 → 转圈超调) */
                {
                    SpeedLoop_Run(&g_sl, speed_ref, (float)enc, SPD_DT);
                }
                g_sl_iq_ref_out = g_sl.iq_ref;
                motor.speed = g_sl.speed;
            }

            /* 速度/位置模式: 每个 1ms 周期都应用速度环最新输出
             * (速度环每 10ms 算一次, 输出持续保持; 之前只在 SPD_DIV 那个周期赋值,
             *  导致 9/10 时间 iq_ref 退回 g_iq_ref → 速度环几乎失效 → 推力小) */
            if (g_foc_mode == FOC_MODE_SPEED || g_foc_mode == FOC_MODE_POSITION)
            {
                iq_ref = g_spd_sign * g_sl.iq_ref;  /* 方向符号修正 */
            }

            /* 失速保护: 仅速度环启用 (SPEED=4)
             * 位置环(5)到位后电机本来就不动, 不能误触发, 所以排除 */
            if (g_foc_mode == FOC_MODE_SPEED)
            {
                static uint16_t stall_enc = 0;
                static uint32_t stall_cnt = 0;
                int32_t delta = (int32_t)enc - (int32_t)stall_enc;
                if (delta >= -200 && delta <= 200) {
                    stall_cnt++;
                    if (stall_cnt > 5000) {   // 5000 × 1ms = 5s
                        g_foc_mode = FOC_MODE_DISABLED;
                        stall_cnt = 0;
                    }
                } else {
                    stall_enc = enc;
                    stall_cnt = 0;
                }
            }

            CurrentLoop_Run(&g_cl, g_id_ref, iq_ref, id_filt, iq_filt, CUR_DT);
            g_iq_ref_cmd = iq_ref;
            motor.vd = g_cl.vd;
            motor.vq = g_cl.vq;
        }
        break;

    default:
        motor.vq = 0.0f;
        motor.vd = 0.0f;
        motor.theta = 0.0f;
        break;
    }

    DQ_t v_dq = { .d = motor.vd / VBUS_HALF, .q = motor.vq / VBUS_HALF};
    InvPark_Run(&v_dq, motor.theta, &v_ab);
    SVPWM_Run(v_ab.alpha, v_ab.beta, &pwm);
    BSP_PWM_SetDuty(pwm.duty_a, pwm.duty_b, pwm.duty_c);
    BSP_LED_FocTick();

}

void FOC_SetMode(FOC_Mode_t mode)
{
    g_foc_mode = mode;
    align_cnt = 0;

    CurrentLoop_Reset(&g_cl);
    SpeedLoop_Reset(&g_sl);
    PositionLoop_Reset(&g_pl);
}


FOC_Mode_t FOC_GetMode(void)
{
    return g_foc_mode;
}

float FOC_GetCurRot(void)
{
    /* 探针1 (u0) 的方向角 = 电流测量在 theta=0 时看到的坐标旋转 (rad) */
    return atan2f(cal_u0b, cal_u0a);
}

float FOC_GetPosSpeedRef(void)
{
    /* 位置环当前输出的速度指令 (rad/s) — 诊断位置环是否在正确出力 */
    return g_pl.speed_ref;
}

/* ===================================================================
 * PI 增益在线调节 (串口命令: clkp/clki/spdkp/spdki)
 * =================================================================== */

void FOC_SetCurrentGains(float kp, float ki)
{
    CurrentLoop_SetGains(&g_cl, kp, ki, kp, ki);
}

void FOC_SetSpeedGains(float kp, float ki)
{
    SpeedLoop_SetGains(&g_sl, kp, ki);
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

void FOC_PositionReset(void)
{
    PositionLoop_Reset(&g_pl);
}