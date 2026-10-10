#ifndef BSP_ADC_H
#define BSP_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void BSP_ADC_Start(void);

/* 清除注入组 EOC/EOS 标志
 * (注入 EOC 中断已关闭, 改由 FOC 循环代替 ISR 清除) */
void BSP_ADC_ClearFlags(void);

uint16_t BSP_ADC_GetPhaseA(void);
uint16_t BSP_ADC_GetPhaseB(void);
uint16_t BSP_ADC_GetPhaseC(void);

#ifdef __cplusplus
}
#endif

#endif