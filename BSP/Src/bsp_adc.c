#include "bsp_adc.h"

extern ADC_HandleTypeDef hadc1;

void BSP_ADC_Start(void)
{
    HAL_ADCEx_InjectedStart_IT(&hadc1);
}

uint16_t BSP_ADC_GetPhaseA(void)
{
    return HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
}

uint16_t BSP_ADC_GetPhaseB(void)
{
    return HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
}

uint16_t BSP_ADC_GetPhaseC(void)
{
    return HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
}