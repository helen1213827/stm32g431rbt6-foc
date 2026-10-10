/**
 * @file    bsp_cmd.c
 * @brief   串口命令接口 (USART3 RX, 115200-8-N-1)
 *
 * 在串口终端输入命令控制 FOC, 无需调试器:
 *   mode <0-5>   切换模式 (0=DISABLED 1=ALIGN 2=VOLTAGE 3=CURRENT 4=SPEED 5=POSITION)
 *   spd  <x>     速度环目标 (rad/s)
 *   iq   <x>     q 轴电流指令 (A)
 *   id   <x>     d 轴电流指令 (A)
 *   off  <x>     电零点偏移 (rad)
 *   isign <+/-1> q 轴电流反馈符号
 *   ssign <+/-1> 速度环输出符号
 *   rs <ohm>     相电阻 R (Ω)           ┐
 *   ls <H>       相电感 L (H)           ├ 改动后自动重算 Kp=ωc·L, Ki=ωc·R
 *   bw <rad/s>   电流环带宽 ωc (rad/s)  ┘
 *   flux <Wb>    永磁磁链 ψf (反电动势前馈用)
 *   ff <0~1>     解耦/反电动势前馈强度 (0=关, 逐步加到1)
 *   vbus <V>     母线电压 (用于矢量圆限幅)
 *   dtc <duty>   死区补偿占空比 (0=关, FD6288T≈0.047)
 *   param        打印当前整定参数
 *   dbg <0-5>    调试打印分组 (0=关 1=电流 2=测速 3=速度 4=位置 5=全量)
 *   prd <ms>     打印周期 (默认 500ms)
 *   stall <0|1>  失速保护开关 (默认 0=关, 调参时建议关)
 *   isr          打印 FOC 中断占用 (last/avg/max, 单位 us)
 *   iql <A>      速度环输出限幅 = 最大 iq (默认 2.0) —— 直接决定机器能给多大加速度
 *   posv <rad/s>    位置环最大速度指令 (默认 5)
 *   posacc <rad/s2> 位置环可用刹车加速度 (默认 100), 决定制动距离 v²/(2a)
 *   posjerk <rad/s3> 参考轨迹加加速度上限 (默认 1000) —— 越小起步/停车越丝滑
 *   help  /  ?   显示帮助
 */

#include "bsp_cmd.h"
#include "usart.h"
#include "foc.h"
#include "motor.h"
#include "current.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

/* 调试打印周期 (ms), 由 prd 命令改。默认 500ms = 2Hz, 人看得过来。
 * 想看波形时 prd 100; 想清静时 dbg 0 或 prd 2000 */
uint32_t g_prd_ms = 50U;

/* 文本打印宏 (保留, 后续如需抑制特定输出可改) */
#define CMD_PRINT(...) do { printf(__VA_ARGS__); } while (0)

/* 命令缓冲区长度 (接收行) */
#define CMD_BUF_LEN 32

static volatile char    cmd_buf[CMD_BUF_LEN];
static volatile uint8_t cmd_len = 0;
static volatile uint32_t last_rx_tick = 0;
volatile uint32_t g_rx_count = 0;   /* 收到的字节计数 (诊断) */
volatile uint32_t g_rx_drop  = 0;   /* 环形缓冲溢出丢弃数 (诊断) */

/* ===================================================================
 * UART RX 环形缓冲
 *
 * 为什么需要: 主循环每 100ms 会被 printf 阻塞 ~7ms (115200 下 80 字符
 * ≈ 7ms)。这期间 USART3 的 RX 寄存器只能存 1 个字节, 后面到达的字符
 * 直接丢掉 → 命令被截断/解析失败。
 *
 * 解决: 把"搬字节进缓冲区"放到 FOC 中断里 (每 100us 一次, 丢不了),
 * 主循环只负责解析。中断里只多几个寄存器的开销。
 * =================================================================== */
#define CMD_RX_BUF_LEN 128

static volatile uint8_t  rx_buf[CMD_RX_BUF_LEN];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

/* 把一字节推进环形缓冲 (USART3 ISR 与 FOC 中断共用) */
static void CMD_RxPush(uint8_t ch)
{
    uint16_t next = (uint16_t)((rx_head + 1U) % CMD_RX_BUF_LEN);

    g_rx_count++;
    if (next == rx_tail)
    {
        g_rx_drop++;            /* 满: 丢新数据 */
    }
    else
    {
        rx_buf[rx_head] = ch;
        rx_head = next;
    }
}

void BSP_CMD_RxTick(void)
{
    /* 保险: 即使 USART3 中断出了问题, FOC 中断也能兵底 */
    while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE))
    {
        CMD_RxPush((uint8_t)(huart3.Instance->RDR & 0xFF));
    }

    /* 清溢出标志, 避免接收卡死 */
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_OREFLAG(&huart3);
    }
}

/* 从环形缓冲取一字节; 无数据返回 -1 */
static int CMD_GetChar(void)
{
    if (rx_tail == rx_head) return -1;

    uint8_t ch = rx_buf[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % CMD_RX_BUF_LEN);
    return (int)ch;
}

void BSP_CMD_Init(void)
{
    /* 打开 USART3 接收中断: 每收到一个字节立即入环形缓冲。
     *
     * 为什么不能只在 FOC 中断里轮询: 115200 波特率一个字节 = 86.8us,
     * 而 FOC 中断 100us 才跑一次 —— 中断比数据还慢, 连续发送必然丢字节
     * (实测发 29 字符丢 2 个)。
     *
     * 注意: USART3 全局中断在 CubeMX 里没开, 所以 stm32g4xx_it.c 里
     * 没有 USART3_IRQHandler, 这里自己定义不会冲突。
     * 优先级 3: 低于 FOC(0)/ADC(1), 高于 SysTick(15)。 */
    HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

    printf("\r\n[CMD] ready v5(rxirq). 一次只发一条命令\r\n");
}

void USART3_IRQHandler(void)
{
    while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE))
    {
        CMD_RxPush((uint8_t)(huart3.Instance->RDR & 0xFF));
    }

    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_OREFLAG(&huart3);
    }
}

static void CMD_Help(void)
{
    CMD_PRINT("mode<0-5> spd<r/s> iq<A> id<A> off<rad> isign/ssign<+/->\r\n");
    CMD_PRINT("clkp clki cl<sp> : 电流环PI  |  spdkp spdki : 速度环PI  |  iql<A> : 最大iq\r\n");
    CMD_PRINT("pos<counts> posrel<counts> poskp poski poskd posv<rad/s> posacc<rad/s2> posjerk<rad/s3>\r\n");
    CMD_PRINT("rs<ohm> ls<H> bw<rad/s> flux<Wb> ff<0-1> vbus<V> dtc<d> param\r\n");
    CMD_PRINT("dbg<0-6> isr : 打印分组 | 0关 1电流 2测速 3速度 4位置 5全量 6原始电流\r\n");
    CMD_PRINT("prd<ms> : 打印周期(默认500)  |  stall<0|1> : 失速保护开关\r\n");
    CMD_PRINT("spdf<ms> : 测速低通时间常数(默认3, 0=关, 1~200)\r\n");
}

/* ===================================================================
 * 调试打印: 按 g_dbg_mode 分组, 避免一行几十个字段看不清
 * 每条行首字母标识分组 (I/S/V/P/A), 末尾 m=当前模式
 * =================================================================== */
void BSP_CMD_PrintDebug(void)
{
    if (g_dbg_mode == 0U) return;

    /* ---- 保护停机提示 ----
     * 保护一触发就会把模式切成 DISABLED, 之后 spd/pos 命令全部失效。
     * 以前只在行尾留一个不起眼的 m=0, 很难发现。这里把原因报出来。 */
    {
        static uint8_t reported = 0;
        uint8_t fc = FOC_GetFaultCode();
        if (fc != 0U && fc != reported)
        {
            reported = fc;
            CMD_PRINT("!!! FAULT %u: %s\r\n",
                      (unsigned)fc,
                      (fc == 1U) ? "KEY 引脚被拉低(悬空? 建议 CubeMX 改 Pull-up)"
                    : (fc == 2U) ? "失控: 转速远超指令(反电动势顶到母线)"
                    :              "失速: 速度模式下堵转");
            CMD_PRINT("!!! 输出已断, 重发一次 mode 命令即可恢复\r\n");
        }
        else if (fc == 0U)
        {
            reported = 0;
        }
    }

    const unsigned long t    = (unsigned long)FOC_IsrLoadAvgUs();   /* ISR 占用 us */
    const unsigned      e    = (unsigned)g_enc_cache;
    const float         mech = (float)g_enc_cache * 0.0000958738f;  /* counts → rad */

    /* 位置误差: 环绕修正后的最短路径 */
    float perr = g_pos_ref - (float)g_enc_cache;
    if      (perr >  32768.0f) perr -= 65536.0f;
    else if (perr < -32768.0f) perr += 65536.0f;

    switch (g_dbg_mode)
    {
    case 1U:   /* 电流环: id/idr 是否一致 / iq 是否跟得上 iqr / vq 是否饱和
                * idw/iqw = 本打印窗口内的 [最小 最大] 值:
                *   平均很小但极值跨度很大 → 环路在振荡(被100ms采样掩盖) */
        CMD_PRINT("I id=%+.3f idr=%+.3f idw=[%+.2f %+.2f] iq=%+.3f iqr=%+.3f iqw=[%+.2f %+.2f] vd=%+.2f vq=%+.2f e=%5u t=%2luuS m=%d\r\n",
                  motor.id, g_id_ref, g_id_min, g_id_max,
                  motor.iq, g_iq_ref_cmd, g_iq_min, g_iq_max,
                  motor.vd, motor.vq, e, t, (int)g_foc_mode);
        break;

    case 2U:   /* 测速: spd 是否平滑、方向对、we 是否≈spd×14
                * spdw=[最小 最大] = 本打印窗口内的速度极值跨度。
                *   ★ 电机静止时 spdw 的跨度就是【测速噪声本底】。
                *   速度环 kp 的可调上限 = 允许的力矩噪声 / 这个跨度。
                *   例: 静止时跨度 1 rad/s, 想让力矩噪声 <0.1A → kp < 0.1 */
        CMD_PRINT("S e=%5u mech=%+.4f spd=%+.2f spdw=[%+.2f %+.2f] we=%+.1f t=%2luuS m=%d\r\n",
                  e, mech, motor.speed, g_spd_min, g_spd_max, g_omega_e,
                  t, (int)g_foc_mode);
        FOC_SpdRangeReset();
        break;

    case 3U:   /* 速度环: spd 能不能跟上 rr(斜坡后的真实参考); slr 是速度环原始输出 */
        CMD_PRINT("V spd=%+.2f ref=%+.2f rr=%+.2f iqr=%+.2f slr=%+.2f iq=%+.3f t=%2luuS m=%d\r\n",
                  motor.speed, g_speed_ref, g_spd_ramp, g_iq_ref_cmd,
                  g_sl_iq_ref_out, motor.iq, t, (int)g_foc_mode);
        break;

    case 4U:   /* 位置环
                * pspd = 位置环输出的速度指令 ω_ref; praw = 限幅前的原始值
                * plim = 速度指令限幅 (= posv)
                *   pspd 大但 spd 跟不上 → 【速度环增益不够】, 调大 spdkp */
        CMD_PRINT("P e=%5u ref=%5d err=%+6d pspd=%+.2f praw=%+.2f plim=%+.2f spd=%+.2f m=%d\r\n",
                  e, (int)g_pos_ref, (int)perr, FOC_GetPosSpeedRef(),
                  FOC_GetPosVraw(), FOC_GetPosVlim(), motor.speed, (int)g_foc_mode);
        break;

    case 5U:   /* 全量 */
        CMD_PRINT("A m=%d id=%+.3f iq=%+.3f iqr=%+.3f vd=%+.2f vq=%+.2f spd=%+.2f e=%5u pr=%5d perr=%+6d pspd=%+.2f slr=%+.2f t=%2luuS\r\n",
                  (int)g_foc_mode, motor.id, motor.iq, g_iq_ref_cmd,
                  motor.vd, motor.vq, motor.speed, e, (int)g_pos_ref,
                  (int)perr, FOC_GetPosSpeedRef(), g_sl_iq_ref_out, t);
        break;

    case 6U:   /* 原始电流: 排查相电流采样/映射/偏移 (ALIGN 期间看)
                * 探针1(vd=6,vq=0, 磁场沿A轴) 应: ia≈+I, ib≈ic≈-I/2, al≈+I, be≈0
                * 探针2(vd=0,vq=6, 磁场沿B轴) 应: ia≈0,  ib≈+0.87I, ic≈-0.87I, be≈+I */
        CMD_PRINT("R vd=%+.2f vq=%+.2f ia=%+.3f ib=%+.3f ic=%+.3f al=%+.3f be=%+.3f id=%+.3f iq=%+.3f m=%d\r\n",
                  motor.vd, motor.vq, current.ia, current.ib, current.ic,
                  ia_raw_filt, ib_raw_filt, motor.id, motor.iq, (int)g_foc_mode);
        break;

    default:
        break;
    }

    FOC_IsrLoadReset();    /* 打印后重置峰值 → 看到的是"这一段"的最大值 */
    FOC_CurRangeReset();   /* 同时重置电流极值窗口 */
}

static void CMD_PrintParams(void)
{
    FOC_Tune_t t;
    FOC_GetTune(&t);

    CMD_PRINT("===== 当前整定参数 (默认值见 FOC/Inc/foc_tune.h) =====\r\n");
    CMD_PRINT("电机  : R=%.3f ohm  L=%.2e H  flux=%.5f Wb  vbus=%.2f V\r\n",
              t.mot_rs, t.mot_ls, t.mot_flux, t.vbus);
    CMD_PRINT("电流环: bw=%.0f rad/s  Kp=%.3f  Ki=%.0f  | ff=%.2f dtc=%.4f\r\n",
              t.cur_bw, t.cur_kp, t.cur_ki, t.ff, t.dtc);
    CMD_PRINT("速度环: kp=%.4f  ki=%.4f  iql=%.2fA  | spdf=%.1fms  win=%lums\r\n",
              t.spd_kp, t.spd_ki, t.spd_iql, t.spd_lpf, (unsigned long)t.spd_win_ms);
    CMD_PRINT("位置环: kp=%.5f  ki=%.5f  kd=%.4f\r\n",
              t.pos_kp, t.pos_ki, t.pos_kd);
    CMD_PRINT("轨迹  : vmax=%.2f rad/s  acc=%.1f rad/s2  jerk=%.0f rad/s3\r\n",
              t.pos_vmax, t.pos_acc, t.pos_jerk);
}

/* 手写浮点解析: 避免 nano libc 的 sscanf %f 未链接(_scanf_float)导致解析失败 */
static float CMD_ParseFloat(const char *s)
{
    float sign = 1.0f, val = 0.0f, frac = 0.1f;
    int   has = 0;

    if (*s == '-')       { sign = -1.0f; s++; }
    else if (*s == '+')  { s++; }

    while (*s >= '0' && *s <= '9') { val = val * 10.0f + (float)(*s - '0'); s++; has = 1; }
    if (*s == '.')
    {
        s++;
        while (*s >= '0' && *s <= '9') { val += (float)(*s - '0') * frac; frac *= 0.1f; s++; has = 1; }
    }
    return has ? (sign * val) : 0.0f;
}

/* 执行一行命令 (line 以 '\0' 结尾) */
static void CMD_Execute(const char *line)
{
    CMD_PRINT(">> %s\r\n", line);   /* 回显原始内容, 便于确认 RX 是否收到 */

    const char *p = line;
    while (*p == ' ' || *p == '\t') p++;

    char cmd[8];
    int i = 0;
    while (*p && *p != ' ' && *p != '\t' && i < (int)sizeof(cmd) - 1) cmd[i++] = *p++;
    cmd[i] = '\0';

    while (*p == ' ' || *p == '\t') p++;
    int   has_arg = (*p != '\0');
    float arg     = CMD_ParseFloat(p);

    if      (strcmp(cmd, "mode") == 0  && has_arg)  { FOC_SetMode((FOC_Mode_t)(int)arg);          CMD_PRINT("mode=%d\r\n", (int)g_foc_mode); }
    else if (strcmp(cmd, "spd")  == 0  && has_arg)  { g_speed_ref = arg;                   CMD_PRINT("spd=%.2f\r\n", g_speed_ref); }
    else if (strcmp(cmd, "iq")   == 0  && has_arg)  { g_iq_ref = arg;                      CMD_PRINT("iq=%.3f\r\n", g_iq_ref); }
    else if (strcmp(cmd, "id")   == 0  && has_arg)  { g_id_ref = arg;                      CMD_PRINT("id=%.3f\r\n", g_id_ref); }
    else if (strcmp(cmd, "off")  == 0  && has_arg)  { g_off = arg;                         CMD_PRINT("off=%.3f\r\n", g_off); }
    else if (strcmp(cmd, "isign") == 0 && has_arg)  { g_iq_sign  = (arg > 0) ? 1.0f : -1.0f; CMD_PRINT("isign=%.1f\r\n", g_iq_sign); }
    else if (strcmp(cmd, "ssign") == 0 && has_arg)  { g_spd_sign = (arg > 0) ? 1.0f : -1.0f; CMD_PRINT("ssign=%.1f\r\n", g_spd_sign); }
    else if (strcmp(cmd, "clkp")  == 0 && has_arg)  { FOC_SetCurrentGains(arg, -1.0f);     CMD_PRINT("clkp=%.3f (ki 不变)\r\n", arg); }
    else if (strcmp(cmd, "clki")  == 0 && has_arg)  { FOC_SetCurrentGains(-1.0f, arg);     CMD_PRINT("clki=%.3f (kp 不变)\r\n", arg); }
    else if (strcmp(cmd, "cl")    == 0 && has_arg)  { FOC_SetCurrentGains(arg, arg);       CMD_PRINT("cl kp=ki=%.3f\r\n", arg); }
    else if (strcmp(cmd, "spdkp") == 0 && has_arg)  { FOC_SetSpeedGains(arg, -1.0f);       CMD_PRINT("spdkp=%.3f (ki 不变)\r\n", arg); }
    else if (strcmp(cmd, "spdki") == 0 && has_arg)  { FOC_SetSpeedGains(-1.0f, arg);       CMD_PRINT("spdki=%.3f (kp 不变)\r\n", arg); }
    else if (strcmp(cmd, "iql")   == 0 && has_arg)  { FOC_SetSpeedLimit(arg);              CMD_PRINT("iql=%.2f A\r\n", arg); }
    else if (strcmp(cmd, "pos")   == 0  && has_arg)  { g_pos_ref = arg; FOC_PositionReset(); CMD_PRINT("pos=%.0f\r\n", g_pos_ref); }
    else if (strcmp(cmd, "posrel")== 0  && has_arg)  { float t = (float)g_enc_cache + arg;
                                                        while (t >= 65536.0f) t -= 65536.0f;
                                                        while (t <  0.0f)     t += 65536.0f;
                                                        g_pos_ref = t; FOC_PositionReset();
                                                        CMD_PRINT("pos=%.0f (rel %+.0f)\r\n", g_pos_ref, arg); }
    else if (strcmp(cmd, "poskp") == 0 && has_arg)  { FOC_SetPositionGains(arg, -1.0f);   CMD_PRINT("poskp=%.3f (ki 不变)\r\n", arg); }
    else if (strcmp(cmd, "poski") == 0 && has_arg)  { FOC_SetPositionGains(-1.0f, arg);   CMD_PRINT("poski=%.3f (kp 不变)\r\n", arg); }
    else if (strcmp(cmd, "poskd") == 0 && has_arg)  { FOC_SetPositionDamping(arg);        CMD_PRINT("poskd=%.3f\r\n", arg); }
    else if (strcmp(cmd, "posv")  == 0 && has_arg)  { FOC_SetPositionVmax(arg); CMD_PRINT("posv=%.2f rad/s (max %.2f)\r\n", arg, FOC_GetPositionVref()); }
    else if (strcmp(cmd, "posacc")== 0 && has_arg)  { FOC_SetPositionAcc(arg);  CMD_PRINT("posacc=%.1f rad/s2\r\n", arg); }
    else if (strcmp(cmd, "posjerk")==0 && has_arg)  { FOC_SetPositionJerk(arg); CMD_PRINT("posjerk=%.0f rad/s3\r\n", arg); }
    else if (strcmp(cmd, "rs")    == 0 && has_arg)  { FOC_SetMotorParams(arg, -1.0f, -1.0f); CMD_PrintParams(); }
    else if (strcmp(cmd, "ls")    == 0 && has_arg)  { FOC_SetMotorParams(-1.0f, arg, -1.0f); CMD_PrintParams(); }
    else if (strcmp(cmd, "flux")  == 0 && has_arg)  { FOC_SetMotorParams(-1.0f, -1.0f, arg); CMD_PrintParams(); }
    else if (strcmp(cmd, "bw")    == 0 && has_arg)  { FOC_SetCurrentBandwidth(arg);        CMD_PrintParams(); }
    else if (strcmp(cmd, "ff")    == 0 && has_arg)  { FOC_SetFeedforward(arg);             CMD_PRINT("ff=%.2f\r\n", arg); }
    else if (strcmp(cmd, "vbus")  == 0 && has_arg)  { FOC_SetVbus(arg);                    CMD_PRINT("vbus=%.2f\r\n", arg); }
    else if (strcmp(cmd, "dtc")   == 0 && has_arg)  { FOC_SetDeadtimeComp(arg);            CMD_PRINT("dtc=%.4f\r\n", arg); }
    else if (strcmp(cmd, "param") == 0)             { CMD_PrintParams(); }
    else if (strcmp(cmd, "prd")   == 0 && has_arg)  { if (arg >= 10.0f && arg <= 5000.0f) g_prd_ms = (uint32_t)arg;
                                                       CMD_PRINT("prd=%lums\r\n", (unsigned long)g_prd_ms); }
    else if (strcmp(cmd, "stall") == 0 && has_arg)  { g_stall_enable = (arg > 0.0f) ? 1U : 0U;
                                                       CMD_PRINT("stall=%u (1=开)\r\n", (unsigned)g_stall_enable); }
    else if (strcmp(cmd, "spdf")  == 0 && has_arg)  { if (arg >= 0.0f && arg <= 200.0f) g_spd_lpf_ms = arg;
                                                       CMD_PRINT("spdf=%.1f ms (0=关)\r\n", g_spd_lpf_ms); }
    else if (strcmp(cmd, "dbg")   == 0 && has_arg)  { g_dbg_mode = (uint8_t)(int)arg; CMD_PRINT("dbg=%d\r\n", (int)g_dbg_mode); }
    else if (strcmp(cmd, "isr")   == 0)             { CMD_PRINT("isr last=%luuS avg=%luuS max=%luuS\r\n",
                                                                     (unsigned long)FOC_IsrLoadLastUs(),
                                                                     (unsigned long)FOC_IsrLoadAvgUs(),
                                                                     (unsigned long)FOC_IsrLoadMaxUs());
                                                       FOC_IsrLoadReset(); }
    else if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) { CMD_Help(); }
    else { CMD_PRINT("? '%s' (try 'help')\r\n", line); }
}

void BSP_CMD_Poll(void)
{
    uint32_t now = HAL_GetTick();
    int      got_byte = 0;
    int      ch;

    /* 从环形缓冲取字节 (搬运在 BSP_CMD_RxTick, ISR 里完成) */
    while ((ch = CMD_GetChar()) >= 0)
    {
        if (ch == '\r' || ch == '\n')
        {
            if (cmd_len > 0)
            {
                cmd_buf[cmd_len] = '\0';
                cmd_len = 0;
                CMD_Execute((char *)cmd_buf);
            }
        }
        else if (cmd_len < CMD_BUF_LEN - 1)
        {
            cmd_buf[cmd_len++] = (char)ch;
            got_byte = 1;
        }
    }
    if (got_byte) last_rx_tick = now;

    /* 超时判定命令结束: 收到字符后 50ms 无新字节 → 视为一行命令
     * (兼容不发 \r\n 的串口工具) */
    if (cmd_len > 0 && (now - last_rx_tick) > 50)
    {
        cmd_buf[cmd_len] = '\0';
        cmd_len = 0;
        CMD_Execute((char *)cmd_buf);
    }
}
