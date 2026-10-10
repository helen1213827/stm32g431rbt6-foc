#ifndef BSP_CMD_H
#define BSP_CMD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化串口命令接口 (USART3 RX, 115200)
 *        在串口终端输入命令控制 FOC, 无需调试器.
 */
void BSP_CMD_Init(void);

/**
 * @brief 轮询解析命令 (在主循环调用)
 */
void BSP_CMD_Poll(void);

/**
 * @brief 把 UART 收到的字节搬进环形缓冲 (在 FOC 中断里调用, 10kHz)
 *
 * 主循环每次 printf 会被阻塞 ~7ms (115200 波特率下 80 字符),
 * 期间 USART3 的 RX 寄存器只能存 1 字节, 后面的字符会丢 → 命令被截断。
 * 放进 10kHz 中断后每 100us 搬一次, 不会再丢。
 */
void BSP_CMD_RxTick(void);

/* 环形缓冲溢出丢弃的字节数 (诊断用) */
extern volatile uint32_t g_rx_drop;

/**
 * @brief 按 g_dbg_mode 分组打印调试数据 (在主循环周期调用)
 *
 * dbg 1=电流环  2=测速  3=速度环  4=位置环  5=全量  6=原始电流  0=关闭
 */
void BSP_CMD_PrintDebug(void);

/* 打印周期 (ms), 默认 500。改: prd 100 / prd 2000 */
extern uint32_t g_prd_ms;

/* 收到的串口字节计数 (诊断用, 打印可见) */
extern volatile uint32_t g_rx_count;

#ifdef __cplusplus
}
#endif

#endif
