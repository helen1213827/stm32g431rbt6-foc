#ifndef BSP_LED_H
#define BSP_LED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief LED 指示灯模块
 *
 * LED1 (PC13) — 心跳灯：系统 alive
 * LED2 (PC14) — FOC 循环指示：每 N 次 FOC 翻转
 * LED3 (PC15) — 故障灯：过流/错误时亮
 */

void BSP_LED_Init(void);

/* ---- 心跳灯 ---- */
void BSP_LED_Heartbeat(void);    /* 在主循环中调用，自动闪烁 */

/* ---- FOC 循环指示 ---- */
void BSP_LED_FocTick(void);      /* 在 FOC_Run 中调用 */

/* ---- 故障灯 ---- */
void BSP_LED_FaultOn(void);
void BSP_LED_FaultOff(void);

#ifdef __cplusplus
}
#endif

#endif // BSP_LED_H
