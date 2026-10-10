#include "foc.h"
#include "foc_tune.h"      /* ★ 所有整定参数的默认值都在这里 */
#include "foc_internal.h"  /* 本模块内部跨文件共享的全局量 */
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
#include "bsp_cmd.h"
#include <math.h>
#include <stdint.h>


/* VBUS_NOMINAL / VBUS_HALF / VBUS_LINEAR_MAX / FOC_FREQ_HZ / FOC_DT 在 foc.h 中定义。
 * 下面所有周期与分频都由 FOC_FREQ_HZ 派生, 改控制频率时只动 foc.h 里那一个数。 */
#define CUR_DT  (FOC_DT)                    /* 电流环 = FOC 节拍 */
#define SPD_DT  (0.001f)                    /* 速度环固定 1kHz */
#define POS_DT  (0.01f)                     /* 位置环固定 100Hz */
#define SPD_DIV (FOC_FREQ_HZ / 1000U)       /* 速度环分频: FOC → 1kHz */
#define POS_DIV (FOC_FREQ_HZ / 100U)        /* 位置环分频: FOC → 100Hz */
#define SPD_RAMP_RATE (20.0f)  /* 速度参考斜坡 rad/s² (仅速度模式用) */
/* 注意: 斜坡从 0 开始, 误差小 → 力矩也小。
 * 云台电机静摩擦/齿槽较大时, 斜坡过慢会让电机起不来。 */

/* ---- 测速: 位置差分滑窗 ----
 * 相对旧的"1 拍差分 + α=0.4 低通":
 *   - 量化噪声÷√N, 低速不再被 1 count 抖动主导
 *   - 群延迟从 ~15ms 降到 ~N·dt/2
 * 窗口按【时间】定义: 改 FOC 频率时测速分辨率不会跟着变差 */
/* 测速滑窗长度 (ms)。
 * ⚠️ 窗口越短, 编码器量化噪声被放大越多:
 *     1 LSB 对应的速度 = (2π/65536) / T
 *     T=4ms  → 2.4 rad/s    ← 噪声比信号还大, 环路根本没法调
 *     T=10ms → 0.96 rad/s
 * 所以这个窗口不能短。想提高带宽必须换【测速方法】
 * (定时器捕获法/周期法), 而不是把窗口裁短。 */
#define SPD_WIN_MS      (FOC_TUNE_SPD_WIN_MS)
#define SPD_WIN         (SPD_WIN_MS * FOC_FREQ_HZ / 1000U)

/* ---- 测速低通时间常数 (ms), 可用 spdf 命令在线改 ----
 *
 * ⚠️ 必须按【时间】定义! 原来写死 α=0.5 是按 1kHz 定的,
 * 升到 10kHz 后 τ = dt·(1-α)/α 只有 0.1ms —— 等于没滤波。
 *
 * 后果很具体: 滑窗测速用的是"窗口首尾两个采样点之差", 噪声 = √2×编码器
 * 抖动(≈±20counts) → ≈0.27 rad/s。位置环的死区 = 噪声/Kp = 0.27/0.001
 * = 270 counts —— 与实测振荡幅度 ±270counts 完全一致。
 *
 * 同时它是拉锯战: 调大 → 噪声小、位置环能收敛, 但速度环相位滞后变大;
 * 调小 → 速度环快, 但位置环被噪声卡死。 */
float g_spd_lpf_ms = 15.0f;    /* 测速低通时间常数 (ms), spdf 可调; 0=关 */

/* 测速噪声本底: 打印窗口内 spd 的极值跨度 (dbg 2 显示)。
 * 电机静止时 spdw 的跨度就是测速噪声, 速度环 kp 的上限由它决定。 */
float g_spd_min = 0.0f;
float g_spd_max = 0.0f;
void FOC_SpdRangeReset(void)
{
    g_spd_min = motor.speed;
    g_spd_max = motor.speed;
}

/* ---- 时间常量 (同样按时间定义, 不写死拍数) ---- */
#define ALIGN_TIME_MS   (3000U)                                /* 对齐时长 3s */
#define ALIGN_TIME      (ALIGN_TIME_MS * FOC_FREQ_HZ / 1000U)

/* ---- 失速保护 (仅速度环) ----
 * 每 STALL_WIN_MS 看一次编码器【累计位移】, 位移小于 STALL_MIN_COUNTS
 * 视为"这一窗口基本没动"; 连续 STALL_TRIP_CNT 个窗口才停机。
 *
 * ⚠️ 千万不要写成"每周期位移阈值": 原来写死 ±200 counts/周期,
 * 在 10kHz 下等于 **1830rpm 以下全算失速** → 电机一转就被自己停机。
 * (那个 200 是按 1kHz 定的, 升频后没跟着缩) */
#define STALL_WIN_MS      (500U)
#define STALL_WIN         (STALL_WIN_MS * FOC_FREQ_HZ / 1000U)
#define STALL_TRIP_CNT    (10U)                      /* 10 × 0.5s = 5s */
#define STALL_MIN_COUNTS  ((int32_t)(2.0f * 10430.0f * (float)STALL_WIN_MS / 1000.0f))

CurrentLoop_t  g_cl;     /* 电流环对象 (foc_params.c 也要用) */
SpeedLoop_t    g_sl;     /* 速度环对象 */
PositionLoop_t g_pl;     /* 位置环对象 */

static AlphaBeta_t i_ab,v_ab;
static DQ_t         i_dq;
static SVPWM_t      pwm;

FOC_Mode_t g_foc_mode = FOC_MODE_ALIGN;  /* 上电先对齐(双探针校准), 不能跳过 */
static bool g_calibrated = false;        /* ALIGN 校准是否完成 */

/* ---- 保护停机的原因码 ----
 * 以前保护一触发就静默切 DISABLED: 串口命令全部失效, 但屏幕上
 * 只能看到一个不起眼的 m=0, 调参时就变成"怎么调都没反应"。
 * 现在每处保护都记下原因, 由打印任务报出来。 */
#define FOC_FAULT_NONE      0U
#define FOC_FAULT_KEY       1U
#define FOC_FAULT_OVERSPEED 2U
#define FOC_FAULT_STALL     3U
volatile uint8_t g_fault_code = FOC_FAULT_NONE;

/* 按键消抖: 连续这么多拍读到低电平才算真的按下 (10kHz → 200 拍 = 20ms)。
 * KEY_Pin 在 CubeMX 里配的是 GPIO_NOPULL, 悬空输入会被电机噪声拉低,
 * 没有消抖就会毫无征兆地掉到 DISABLED。 */
#define KEY_TRIP_CNT  200U
float g_vd_open     = 0.0f;
float g_vq_open     = 3.0f;
float g_id_ref      = 0.0f;   /* 默认 0: 不励磁。要做 d 轴锁定测试就 id 0.3 */
float g_iq_ref      = 0.0f;   /* 默认 0: 上电不跑车。要转手动 iq <A> */
float g_off         = 0.0f;   // 校准后的电零点偏移, debugger 可改
float g_align_vd    = 6.0f;   // 开环对齐电压 (12V 母线, 6V 安全)
CurFrame_t g_cur_frame = { 1.0f, 0.0f, 0.0f, 1.0f };  /* 默认单位阵 */
float g_iq_sign      = 1.0f;  /* q 轴电流反馈符号 (d轴正确,q轴反了 → 翻转) */
float g_spd_sign     = -1.0f; /* 速度环输出符号 (iq_ref<0→+mech, 需取反) */
uint16_t g_enc_cache = 0;     // FOC 循环缓存编码器值, 供主循环打印
float g_speed_ref   = 20.0f;   /* 速度环目标 (rad/s, 机械角速度) ≈ 191rpm */
float g_pos_ref     = 500.0f;

/* ===================================================================
 * 电机电气参数 / 电流环整定 (串口可在线改: rs/ls/flux/bw/ff/vbus/dtc)
 * ===================================================================
 * 电流环按零极点对消整定:  Kp = ωc·L,  Ki = ωc·R
 * 关键: Kp 与 Ki 必须来自【同一个 ωc】。
 * 之前 Kp=0.5 对应 ωc=Kp/L≈500rad/s, 而 Ki=5 对应 ωc=Ki/R≈0.7rad/s,
 * 两者自相矛盾 → 积分几乎不起作用 → 稳态误差永远消不掉、
 * 想靠手调 Ki 补又怎么调都别扭。
 * 另外 Ki 上限 CUR_KI_MAX 之前只有 200, 会把整定值直接截断。 */
float g_motor_rs   = FOC_TUNE_RS;    /* 相电阻 R (Ω) */
float g_motor_ls   = FOC_TUNE_LS;    /* 相电感 L (H) */
float g_motor_flux = FOC_TUNE_FLUX;  /* 永磁磁链 ψf (Wb) */
float g_cur_bw     = FOC_TUNE_CUR_BW;/* 电流环目标带宽 ωc (rad/s) */
float g_ff_gain    = FOC_TUNE_FF;    /* 解耦/反电动势前馈强度 0~1 */
float g_vbus       = FOC_TUNE_VBUS;  /* 母线电压 (V) */
float g_dt_comp    = FOC_TUNE_DTC;   /* 死区补偿占空比 */
float g_omega_e    = 0.0f;      /* 估计的电角速度 (rad/s), 供解耦前馈 */

/* 串口调试打印分组 (dbg 命令切换)
 * 0=关 1=电流环 2=测速 3=速度环 4=位置环 5=全量 */
uint8_t g_dbg_mode = 1U;

uint32_t align_cnt = 0;
/* ALIGN_TIME / STALL_COUNT 已在上方由 FOC_FREQ_HZ 派生 */

/* 打印窗口内的电流极值: 用来发现被 100ms 采样掩盖的高频振荡。
 * 如果 id/iq 的瞬时值看起来很小, 但极值跨度很大 → 环路在振荡。 */
float g_id_min = 0.0f, g_id_max = 0.0f;
float g_iq_min = 0.0f, g_iq_max = 0.0f;

/* 电流低通滤波 (仅打印/观测用) */
float g_id_filt = 0.0f;
float g_iq_filt = 0.0f;

/* 电流指令 (iq_ref) 打印用, 诊断速度环 vs 电流环 */
float g_iq_ref_cmd = 0.0f;
float g_sl_iq_ref_out = 0.0f;  /* 速度环原始输出 */
/* 未补偿的静止系电流滤波, 供 ALIGN 双探针标定 (dbg 6 可观测原始值) */
float ia_raw_filt = 0.0f;
float ib_raw_filt = 0.0f;
/* 探针1 (vd) 标定的 u0 */
float cal_u0a = 1.0f, cal_u0b = 0.0f;
/* 失速保护开关: 0=关(默认)。
 * 调速度环时建议先关掉 —— 否则"电机还没被转起来"就会被它判失速停机,
 * 跟调参互相干扰。调好了再用 stall 1 打开。 */
uint8_t g_stall_enable = 0U;

/* 速度参考斜坡状态 (全局: 切换模式时会清零, 也供 dbg 3 打印) */
float g_spd_ramp = 0.0f;

/* ===================================================================
 * Park / InvPark 共用同一个 theta —— 每周期只算一次 sin/cos
 * 原来两处各算一遍 sinf+cosf = 4 次超越函数调用/周期,
 * 在 -O0 下这是 FOC 中断里最大的一块开销。
 * =================================================================== */
static float g_sin_theta  = 0.0f;
static float g_cos_theta  = 1.0f;
static float g_trig_theta = 0.0f;
static bool  g_trig_valid = false;

static void FOC_SyncTrig(float theta)
{
    if (!g_trig_valid || theta != g_trig_theta)
    {
        g_trig_theta = theta;
        g_sin_theta  = sinf(theta);
        g_cos_theta  = cosf(theta);
        g_trig_valid = true;
    }
}

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

    FOC_SyncTrig(motor.theta);
    Park_Run(&i_ab, g_sin_theta, g_cos_theta, &i_dq);
    motor.id = i_dq.d;
    motor.iq = g_iq_sign * i_dq.q;   /* q 轴符号修正: 使 q 环为负反馈 */

    /* 滤波值【只用于打印/观测】。
     * 之前 id_filt/iq_filt (α=0.1 @1kHz → τ≈9ms) 直接进了电流环反馈,
     * 相当于在闭环里插了一个 ~18Hz 的低通: 相位滞后把电流环带宽压死,
     * Ki 一加上去就振荡。闭环内环一律用瞬时值。 */
    g_id_filt += 0.1f * (motor.id - g_id_filt);
    g_iq_filt += 0.1f * (motor.iq - g_iq_filt);

    /* 窗口极值统计 */
    if (motor.id < g_id_min) g_id_min = motor.id;
    if (motor.id > g_id_max) g_id_max = motor.id;
    if (motor.iq < g_iq_min) g_iq_min = motor.iq;
    if (motor.iq > g_iq_max) g_iq_max = motor.iq;
}

void FOC_CurRangeReset(void)
{
    g_id_min = g_id_max = motor.id;
    g_iq_min = g_iq_max = motor.iq;
}

/* ===================================================================
 * 测速 / 前馈 / 中断优先级
 * =================================================================== */

static uint16_t spd_hist[SPD_WIN];
static uint8_t  spd_idx    = 0;
static uint8_t  spd_fill   = 0;
static bool     spd_primed = false;
static float    spd_lpf    = 0.0f;

/* 1kHz 位置差分滑窗 → 机械角速度 (rad/s) */
static float FOC_EstimateSpeed(uint16_t enc)
{
    if (!spd_primed)
    {
        for (uint32_t i = 0; i < SPD_WIN; i++) spd_hist[i] = enc;
        spd_idx    = 0;
        spd_fill   = 1;
        spd_lpf    = 0.0f;
        spd_primed = true;
        return 0.0f;
    }

    uint16_t oldest = spd_hist[spd_idx];
    spd_hist[spd_idx] = enc;
    spd_idx = (uint8_t)((spd_idx + 1U) % SPD_WIN);
    if (spd_fill < SPD_WIN) spd_fill++;

    int32_t d = (int32_t)enc - (int32_t)oldest;
    if      (d >  32767) d -= 65536;
    else if (d < -32768) d += 65536;

    float dt_win = FOC_DT * (float)spd_fill;   /* 启动阶段窗口未填满 */
    float raw    = (float)d * (6.2831853f / 65536.0f) / dt_win;

    /* 一阶低通: α 按【时间常数】算 (不能写死, 否则改 FOC 频率就废)。
     * g_spd_lpf_ms = 0 表示不滤波 (α=1) —— 调高速度环增益时先关掉它。 */
    float alpha = (g_spd_lpf_ms > 0.0f)
                ? (FOC_DT / (FOC_DT + g_spd_lpf_ms * 0.001f))
                : 1.0f;
    spd_lpf += alpha * (raw - spd_lpf);
    return spd_lpf;
}

/* dq 解耦 + 反电动势前馈
 *   vd_ff = -ωe·Lq·iq
 *   vq_ff = +ωe·Ld·id + ωe·ψf
 * 主要负担交给前馈后, PI 的 Ki 只需扫尾 → 可以又小又稳。
 * g_ff_gain 默认 0 (关闭), 确认符号/方向后再逐步加到 1。 */
static void FOC_ComputeFeedforward(float id, float iq, float *vd_ff, float *vq_ff)
{
    if (g_ff_gain <= 0.0f)
    {
        *vd_ff = 0.0f;
        *vq_ff = 0.0f;
        return;
    }

    float we = g_omega_e * g_ff_gain;
    *vd_ff = -we * g_motor_ls * iq;
    *vq_ff =  we * g_motor_ls * id + we * g_motor_flux;
}

/* 电流环整定
 *
 * 连续域 PI:  v = Kp·e + Ki·∫e dt,  被控对象 1/(Ls+R)
 * 零极点对消的设计值:  Kp = ωc·L,  Ki = ωc·R
 *
 * 但本电机 L/R ≈ 0.2ms ≪ 采样周期 1ms —— 在 1kHz 下电感已经"看不见",
 * 被控对象退化成纯阻性增益 1/R (实测 I ≈ V/5Ω)。此时 ωc·L 只有 0.4,
 * 占环路增益的比例微乎其微, 电流环几乎全靠积分 → 表现就是"软"。
 * 所以给 Kp 加一个 0.3·R 的下限: 让比例路径在直流附近贡献 ~0.3 的环路增益;
 * Ki 仍由 ωc·R 决定穿越频率。
 * (离散极点校验: R=5Ω, Kp=1.5, Ki=2000 → |λ|≈0.47, 无过冲) */
void FOC_ApplyCurrentGains(void)
{
    float kp       = g_cur_bw * g_motor_ls;
    float kp_floor = 0.3f * g_motor_rs;    /* 阻性主导时的比例下限 */

    if (kp < kp_floor) kp = kp_floor;

    FOC_SetCurrentGains(kp, g_cur_bw * g_motor_rs);
}

/* 中断优先级: FOC (TIM6) 必须严格高于其它 HAL 驱动中断。
 * HAL_Init 默认 NVIC_PRIORITYGROUP_4 (4bit 全为抢占位), 数值越小越高。
 * 若与 DMA/CAN/USB/ADC 同优先级则不能互相抢占 → HAL 的长中断会把
 * 1ms 的 FOC 节拍顶住, 造成电流采样抖动。 */
static void FOC_IRQ_Priority_Init(void)
{
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn,      0, 0);   /* FOC 控制节拍: 最高 */
    HAL_NVIC_SetPriority(ADC1_2_IRQn,        1, 0);   /* 电流采样完成 */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 2, 0);
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 2, 0);
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn,    2, 0);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn,    2, 0);
    HAL_NVIC_SetPriority(USB_HP_IRQn,        2, 0);
    HAL_NVIC_SetPriority(USB_LP_IRQn,        2, 0);
    /* SysTick 保持 HAL 默认最低 (15), 不会打断 FOC */
}

/* ===================================================================
 * ISR 占用率测量 (Cortex-M4 DWT 周期计数器)
 * FOC 跑到 10kHz 后最该盯的数字就是"这一拍花了多少 us"。
 * 预算 = FOC_DT = 100us; 动态均值超过 ~60% 就该砍开销。
 * =================================================================== */
static uint32_t s_cyc_per_us = 170U;

volatile uint32_t g_isr_us_last = 0U;   /* 最近一拍 */
volatile uint32_t g_isr_us_max  = 0U;   /* 峰值 (FOC_IsrLoadReset 后可清) */
volatile uint32_t g_isr_us_avg  = 0U;   /* 均值, 每 1000 拍刷新 */

static uint32_t s_isr_cyc_acc = 0U;
static uint32_t s_isr_cyc_cnt = 0U;

static void FOC_IsrLoadInit(void)
{
    s_cyc_per_us = SystemCoreClock / 1000000U;
    if (s_cyc_per_us == 0U) s_cyc_per_us = 170U;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

void FOC_IsrLoadReset(void)
{
    g_isr_us_max = g_isr_us_last;
}

uint32_t FOC_IsrLoadAvgUs(void)  { return g_isr_us_avg;  }
uint32_t FOC_IsrLoadMaxUs(void)  { return g_isr_us_max;  }
uint32_t FOC_IsrLoadLastUs(void) { return g_isr_us_last; }



void FOC_Init(void)
{
    /* 先把 FOC 中断提到最高优先级 (在任何中断源启动之前调用) */
    FOC_IRQ_Priority_Init();
    FOC_IsrLoadInit();

    /* ================= 电流环 ================= */
    CurrentLoop_Init(&g_cl, 1.0f, 1.0f, VBUS_LINEAR_MAX);
    FOC_ApplyCurrentGains();     /* Kp/Ki 由 FOC_TUNE_CUR_BW × R/L 自动算 */

    /* ================= 速度环 (1kHz) =================
     * 参数在 FOC/Inc/foc_tune.h —— 要改就改那里, 或者用串口 spdkp/spdki/iql。
     * kp 的上限由【测速噪声】而非稳定性决定: 力矩噪声 = spdkp × σ_ω,
     * 实测 spdkp > 0.1 就明显振动。先 `dbg 2` 看静止时 spdw 的跨度再定。 */
    SpeedLoop_Init(&g_sl, FOC_TUNE_SPD_KP, FOC_TUNE_SPD_KI, FOC_TUNE_SPD_IQL);

    /* ================= 位置环 (100Hz) =================
     * 参数在 FOC/Inc/foc_tune.h, 算法说明见 position_loop.h。
     * kd: 位置环阻尼。内环几乎纯积分 → 位置 P 环套上去是无阻尼二阶系统,
     *     表现就是"到位后来回摆"。kd 作用在实测速度上提供阻尼,
     *     但会带来落差 (droop = kd × 转速), 而落差又由 ki 去补, 两者成对。 */
    PositionLoop_Init(&g_pl, FOC_TUNE_POS_KP, FOC_TUNE_POS_KI,
                              FOC_TUNE_POS_KD, FOC_TUNE_POS_VMAX);
    PositionLoop_SetAcc (&g_pl, FOC_TUNE_POS_ACC);
    PositionLoop_SetJerk(&g_pl, FOC_TUNE_POS_JERK);

    Angle_Init();
    Angle_SetMode(ANGLE_MODE_ENCODER);
    Current_Init();
}

/* ===================================================================
 * FOC_Run 的组成 —— 一个 10kHz 中断拆成这几块
 *
 *   FOC_Run()            ← 唯一入口 (挂在 TIM6 中断上)
 *     ① 读编码器 + 测速              FOC_EstimateSpeed()
 *     ② 查按键 / 查保护              (按键、失控、失速)
 *     ③ 按模式调用下面四个之一:
 *          FOC_RunDisabled()  模式0 不上电, 只跟踪角度
 *          FOC_RunAlign()     模式1 上电双探针校准
 *          FOC_RunOpenLoop()  模式2 电压开环 (只用于验硬件)
 *          FOC_RunDrive()     模式3/4/5 正常驱动
 *                             └ 位置环(100Hz) → 速度环(1kHz) → 电流环(10kHz)
 *     ④ 把 dq 电压变成三相占空比     FOC_OutputVoltage()
 *     ⑤ 统计 ISR 耗时
 *
 * 读代码建议顺序: FOC_Run → FOC_RunDrive → FOC_OutputVoltage
 * =================================================================== */
static void FOC_RunDisabled(float mech);
static void FOC_RunAlign(void);
static void FOC_RunOpenLoop(float mech);
static void FOC_RunDrive(float mech, float spd_mech);
static void FOC_OutputVoltage(void);


void FOC_Run(void)
{
    uint32_t cyc0 = DWT->CYCCNT;   /* ISR 耗时测量起点 */

    /* 把串口收到的字节搬进环形缓冲 (主循环被 printf 阻塞时会丢字符) */
    BSP_CMD_RxTick();

    /* 按键停机 (消抖版)。
     * ⚠️ 根治办法: CubeMX 里把 KEY 的 Pull 改成 Pull-up (按键按下拉地)。
     * KEY_Pin 现在是 GPIO_NOPULL, 悬空输入会被电机噪声/电刷干扰拉低。 */
    {
        static uint16_t key_cnt = 0;
        if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
        {
            if (key_cnt < KEY_TRIP_CNT) key_cnt++;
            if (key_cnt == KEY_TRIP_CNT)
            {
                g_foc_mode   = FOC_MODE_DISABLED;
                g_fault_code = FOC_FAULT_KEY;
                BSP_LED_FaultOn();
            }
        }
        else
        {
            key_cnt = 0;
        }
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

    /* 1kHz 测速: 解耦前馈 / 速度环反馈 / 位置环阻尼 共用同一个估计值 */
    float spd_mech = FOC_EstimateSpeed(g_enc_cache);
    g_omega_e   = Angle_MechToElecSpeed(spd_mech);
    motor.speed = spd_mech;
    if (spd_mech < g_spd_min) g_spd_min = spd_mech;
    if (spd_mech > g_spd_max) g_spd_max = spd_mech;

    switch(g_foc_mode)
    {
        case FOC_MODE_DISABLED:
            FOC_RunDisabled(mech);
            break;

        case FOC_MODE_ALIGN:
            FOC_RunAlign();
            break;

        case FOC_MODE_VOLATAGE_OPENLOOP:
                FOC_RunOpenLoop(mech);
                break;
        
        case FOC_MODE_CURRENT:
        case FOC_MODE_SPEED:
        case FOC_MODE_POSITION:
            FOC_RunDrive(mech, spd_mech);
            break;

    default:
        motor.vq = 0.0f;
        motor.vd = 0.0f;
        motor.theta = 0.0f;
        break;
    }

    /* ---- 电压输出 ---- */
    FOC_OutputVoltage();

    /* ---- ISR 耗时统计 ---- */
    uint32_t cyc = DWT->CYCCNT - cyc0;
    g_isr_us_last = cyc / s_cyc_per_us;
    if (g_isr_us_last > g_isr_us_max) g_isr_us_max = g_isr_us_last;

    s_isr_cyc_acc += cyc;
    if (++s_isr_cyc_cnt >= 1000U)          /* 1kHz 刷新一次均值 (10kHz 即 100ms) */
    {
        g_isr_us_avg  = (s_isr_cyc_acc / 1000U) / s_cyc_per_us;
        s_isr_cyc_acc = 0U;
        s_isr_cyc_cnt = 0U;
    }
}

/* ===================================================================
 * 模式 0: 不上电
 * 只跟踪角度, 供手动旋转验证编码器方向/接线。
 * =================================================================== */
static void FOC_RunDisabled(float mech)
{
    Angle_Update(mech, CUR_DT);
    motor.theta = Angle_GetTheta();
    motor.vd = 0.0f;
    motor.vq = 0.0f;
}

/* ===================================================================
 * 模式 2: 电压开环
 * 直接把固定 vd/vq 打给电机, 不过任何环路。
 * 只用于验硬件(相序/方向/波形), 不用于正常运行。
 * =================================================================== */
static void FOC_RunOpenLoop(float mech)
{
    Angle_Update(mech, CUR_DT);
    motor.theta = Angle_GetTheta();

    FOC_ReadCurrents();      /* 读电流并变换, 只看不控 */

    motor.vd = g_vd_open;
    motor.vq = g_vq_open;
}

/* ===================================================================
 * 模式 1: 上电校准 (双探针)
 *
 * 目的: 把"控制器认为的角度"和"电机真实磁场方向"对齐。
 *       不做这一步, 电流全打在错误的轴上, 电机推不动甚至发热。
 *
 * 做法: theta 固定为 0, 给两个已知方向的电压, 看电流实际往哪个方向流。
 *   probe1 (前半段): vd=6V, vq=0  → 磁场打在 α 轴, 记录方向 u0
 *   probe2 (后半段): vd=0V, vq=6V  → 磁场打在 β 轴, 记录方向 u1
 *   由 u0/u1 反解出"测量坐标系"到"真实坐标系"的修正矩阵 C。
 *
 * 同时标定电零点偏移 g_off (在 probe1 位置, 转子已静止时测)。
 * 结束后自动进入电流环模式。
 * =================================================================== */
static void FOC_RunAlign(void)
{
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
    if (align_cnt >= ALIGN_TIME)
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
        /* 首次上车先落到【电流环】: 能直接验证电流采样与电流环,
         * 确认无误后再用 mode 4 / mode 5 逐级往上。 */
        FOC_SetMode(FOC_MODE_CURRENT);
    }
}

/* ===================================================================
 * 模式 3/4/5: 正常驱动 —— 三级环路串联
 *
 *   位置环 (100Hz)  → 速度指令 ω_ref
 *        ↓
 *   速度环 (1kHz)   → 电流指令 iq_ref
 *        ↓
 *   电流环 (10kHz)  → 电压 vd/vq   (交给 FOC_OutputVoltage 输出)
 *
 * 每个环都有自己的分频 (POS_DIV / SPD_DIV):
 * 位置环 100Hz、速度环 1kHz、电流环 10kHz 与原 FOC 节拍相同。
 * ⚠️ 分频不能省 —— 增益是按各自 dt 整定的, 频率变了积分速率就变了。
 * =================================================================== */
static void FOC_RunDrive(float mech, float spd_mech)
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

    /* ================= 位置环 (100Hz) =================
     * 环路本身只做 P + 速度前馈; 目标平滑在位置环内部的
     * 参考轨迹发生器里完成, 这里不做任何额外处理。 */
    if (g_foc_mode == FOC_MODE_POSITION && (lo_cnt % POS_DIV) == 0)
    {
        PositionLoop_Run(&g_pl, g_pos_ref, enc, spd_mech, POS_DT);
    }

    if (g_foc_mode == FOC_MODE_POSITION)
    {
        /* 位置环输出直连速度环, 绕过斜坡 */
        speed_ref = g_pl.speed_ref;
    }
    else if (g_foc_mode == FOC_MODE_SPEED)
    {
        /* 速度参考斜坡: 仅速度模式用, 防阶跃超调 */
        float diff = speed_ref - g_spd_ramp;
        float step = SPD_RAMP_RATE * FOC_DT;   /* 用 FOC 实际节拍, 不是 SPD_DT */
        if      (diff >  step) g_spd_ramp += step;
        else if (diff < -step) g_spd_ramp -= step;
        else                   g_spd_ramp  = speed_ref;
        speed_ref = g_spd_ramp;
    }

    /* ================= 速度环 (1kHz) =================
     * 反馈直接用滑窗测速值。
     * 必须按 SPD_DIV 分频: 增益是按 SPD_DT=1ms 整定的。
     * 只在速度/位置模式跑: 电流模式下 iq_ref 由用户直接给。 */
    if ((g_foc_mode == FOC_MODE_SPEED || g_foc_mode == FOC_MODE_POSITION)
        && (lo_cnt % SPD_DIV) == 0)
    {
        SpeedLoop_Run(&g_sl, speed_ref, spd_mech, SPD_DT);
    }
    g_sl_iq_ref_out = g_sl.iq_ref;

    if (g_foc_mode == FOC_MODE_SPEED || g_foc_mode == FOC_MODE_POSITION)
    {
        iq_ref = g_spd_sign * g_sl.iq_ref;  /* 方向符号修正 */
    }

    /* ---- 保护 1: 失控 (转速远超指令) ----
     * 实测转速远超指令 = 反电动势已经顶到母线电压, 电流环失去控制权。
     * 此时速度环刹不住、位置环还在蓄力, 表现就是"轻轻一拨就猛烈乱转"。
     * 持续 100ms 就切 DISABLED 断输出, 重新发一次 mode 命令恢复。 */
    if (g_foc_mode == FOC_MODE_SPEED || g_foc_mode == FOC_MODE_POSITION)
    {
        static uint16_t ovs_cnt = 0;
        /* 限幅取 max(实际参考, 用户设定参考):
         * 刚切模式时斜坡从 0 起, 只看 speed_ref 的话,
         * "电机还在溜车"会被当成失控 → 一上电就报故障。 */
        float ref_abs = fmaxf(fabsf(speed_ref), fabsf(g_speed_ref));
        float ovs_lim = ref_abs + 15.0f;            /* 允许超调 15 rad/s */
        if (spd_mech > ovs_lim || spd_mech < -ovs_lim)
        {
            if (++ovs_cnt >= 1000U)                 /* 10kHz × 1000 = 100ms */
            {
                ovs_cnt = 0;
                FOC_SetMode(FOC_MODE_DISABLED);
                g_fault_code = FOC_FAULT_OVERSPEED;
                BSP_LED_FaultOn();
            }
        }
        else
        {
            ovs_cnt = 0;
        }
    }

    /* ---- 保护 2: 失速 (仅速度模式) ----
     * 位置环到位后电机本来就不动, 不能误触发, 所以排除位置模式。
     * 判定用【窗口累计位移】, 不能用每周期位移 —— 后者会随控制频率失效。 */
    if (g_stall_enable && g_foc_mode == FOC_MODE_SPEED)
    {
        static uint32_t stall_enc   = 0;
        static int32_t  stall_accum = 0;
        static uint32_t stall_win   = 0;
        static uint32_t stall_cnt   = 0;

        int32_t d = (int32_t)enc - (int32_t)stall_enc;
        if      (d >  32767) d -= 65536;
        else if (d < -32768) d += 65536;
        stall_enc = enc;

        if (d > -3000 && d < 3000) stall_accum += d;   /* 丢掉跳变 */

        if (++stall_win >= STALL_WIN)
        {
            stall_win = 0;

            if (stall_accum > -STALL_MIN_COUNTS &&
                stall_accum <  STALL_MIN_COUNTS)
            {
                if (++stall_cnt >= STALL_TRIP_CNT)
                {
                    g_foc_mode   = FOC_MODE_DISABLED;
                    g_fault_code = FOC_FAULT_STALL;
                    BSP_LED_FaultOn();
                    stall_cnt = 0;
                }
            }
            else
            {
                stall_cnt = 0;             /* 真的在转 → 清零 */
            }
            stall_accum = 0;
        }
    }

    /* ================= 电流环 (10kHz) =================
     * 瞬时反馈 + PI + dq 解耦/反电动势前馈 */
    float vd_ff, vq_ff;
    FOC_ComputeFeedforward(motor.id, motor.iq, &vd_ff, &vq_ff);

    CurrentLoop_Run(&g_cl, g_id_ref, iq_ref,
                    motor.id, motor.iq, vd_ff, vq_ff, CUR_DT);
    g_iq_ref_cmd = iq_ref;
    motor.vd = g_cl.vd;
    motor.vq = g_cl.vq;
}

/* ===================================================================
 * 输出级: dq 电压 → αβ → 三相占空比
 *
 *   1) 矢量圆限幅: 限制合成矢量幅值, 不单独限 vd/vq
 *   2) 反 Park       → αβ
 *   3) SVPWM         → 三相占空比
 *   4) 死区补偿      (默认关)
 *   5) 写 PWM 寄存器 + 呼吸灯
 * =================================================================== */
static void FOC_OutputVoltage(void)
{
    /* ---- 电压矢量圆限幅 ----
     * 上限取【SVPWM 线性区】与【占空比 5%~95% 钳位】中更严的那个:
     *   duty = 0.5 + 0.5·(u - offset),  max|u - offset| = 0.866·|u|
     *   duty 摆幅被限在 0.45 → |u|max = 0.45/(0.5·0.866) = 1.039
     *   |v|max = (Vbus/2)·1.039 ≈ 0.52·Vbus  (12V → 6.24V)
     * 旧值 Vbus/√3 = 6.93V 比实际可用电压高 11%, 限不住真正的天花板。 */
    float vmax  = (g_vbus * 0.5f) * (0.45f / (0.5f * 0.8660254f));
    float vmag2 = motor.vd * motor.vd + motor.vq * motor.vq;
    if (vmag2 > vmax * vmax)
    {
        float scale = vmax / sqrtf(vmag2);
        motor.vd *= scale;
        motor.vq *= scale;
    }

    DQ_t v_dq = { .d = motor.vd / VBUS_HALF, .q = motor.vq / VBUS_HALF};
    FOC_SyncTrig(motor.theta);          /* theta 未变 → 直接复用上面的 sin/cos */
    InvPark_Run(&v_dq, g_sin_theta, g_cos_theta, &v_ab);
    SVPWM_Run(v_ab.alpha, v_ab.beta, &pwm);

    /* ---- 死区补偿 (默认关闭) ----
     * FD6288T 死区 200 counts → Td/Ts ≈ 0.047。按相电流符号抬高/压低占空比。
     * 电流方向约定与采样极性有关, 开启前先用小电流静态测试确认符号。 */
    if (g_dt_comp > 0.0f)
    {
        pwm.duty_a += (motor.ia >= 0.0f) ? g_dt_comp : -g_dt_comp;
        pwm.duty_b += (motor.ib >= 0.0f) ? g_dt_comp : -g_dt_comp;
        pwm.duty_c += (motor.ic >= 0.0f) ? g_dt_comp : -g_dt_comp;
    }

    BSP_PWM_SetDuty(pwm.duty_a, pwm.duty_b, pwm.duty_c);
    BSP_LED_FocTick();
}

/* ===================================================================
 * 设置接口 (FOC_Set* / FOC_Get*)
 * → 全部在 foc_params.c, 本文件只放控制逻辑
 * =================================================================== */