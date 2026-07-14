#ifndef BSP_ADC_H
#define BSP_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void BSP_ADC_Start(void);

uint16_t BSP_ADC_GetPhaseA(void);
uint16_t BSP_ADC_GetPhaseB(void);
uint16_t BSP_ADC_GetPhaseC(void);

#ifdef __cplusplus
}
#endif

#endif