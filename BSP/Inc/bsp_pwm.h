#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "main.h"

void BSP_PWM_Init(void);

void BSP_PWM_Start(void);
void BSP_PWM_Stop(void);
void BSP_PWM_SetDuty(float dutyA,float dutyB, float dutyC);
#endif // BSP_PWM_H