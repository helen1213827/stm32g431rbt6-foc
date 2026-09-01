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
 *   help  /  ?   显示帮助
 */

#include "bsp_cmd.h"
#include "usart.h"
#include "foc.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

#define CMD_BUF_LEN 32

/* 文本打印宏 (保留, 后续如需抑制特定输出可改) */
#define CMD_PRINT(...) do { printf(__VA_ARGS__); } while (0)

static volatile char    cmd_buf[CMD_BUF_LEN];
static volatile uint8_t cmd_len = 0;
static volatile uint32_t last_rx_tick = 0;
volatile uint32_t g_rx_count = 0;   /* 收到的字节计数 (诊断) */

void BSP_CMD_Init(void)
{
    printf("\r\n[CMD] ready v3(poll). type 'help'\r\n");
}

static void CMD_Help(void)
{
    CMD_PRINT("mode<0-5> spd<r/s> iq<A> id<A> off<rad> isign/ssign<+/->\r\n");
    CMD_PRINT("clkp clki cl<sp> : 电流环PI  |  spdkp spdki : 速度环PI\r\n");
    CMD_PRINT("pos<counts> poskp poski poskd : 位置环 | help\r\n");
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

    if      (strcmp(cmd, "mode") == 0  && has_arg)  { g_foc_mode = (FOC_Mode_t)(int)arg;   CMD_PRINT("mode=%d\r\n", (int)g_foc_mode); }
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
    else if (strcmp(cmd, "pos")   == 0  && has_arg)  { g_pos_ref = arg; FOC_PositionReset(); CMD_PRINT("pos=%.0f\r\n", g_pos_ref); }
    else if (strcmp(cmd, "poskp") == 0 && has_arg)  { FOC_SetPositionGains(arg, -1.0f);   CMD_PRINT("poskp=%.3f (ki 不变)\r\n", arg); }
    else if (strcmp(cmd, "poski") == 0 && has_arg)  { FOC_SetPositionGains(-1.0f, arg);   CMD_PRINT("poski=%.3f (kp 不变)\r\n", arg); }
    else if (strcmp(cmd, "poskd") == 0 && has_arg)  { FOC_SetPositionDamping(arg);        CMD_PRINT("poskd=%.3f\r\n", arg); }
    else if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) { CMD_Help(); }
    else { CMD_PRINT("? '%s' (try 'help')\r\n", line); }
}

void BSP_CMD_Poll(void)
{
    uint32_t now = HAL_GetTick();
    int      got_byte = 0;

    /* 轮询读取所有待收字节 (USART3 RXNE 寄存器), 不用中断/锁/HAL状态机 */
    while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE))
    {
        g_rx_count++;                       /* 每收到一字节计数 */
        uint8_t ch = (uint8_t)(huart3.Instance->RDR & 0xFF);

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

    /* 清溢出标志, 避免接收卡死 */
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_OREFLAG(&huart3);
    }
}
