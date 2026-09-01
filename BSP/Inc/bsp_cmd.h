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

/* 收到的串口字节计数 (诊断用, 打印可见) */
extern volatile uint32_t g_rx_count;

#ifdef __cplusplus
}
#endif

#endif
