#include "bsp_adc.h"

extern ADC_HandleTypeDef hadc1;

/* ADC Raw Value */
static volatile uint16_t phaseA = 0;
static volatile uint16_t phaseB = 0;
static volatile uint16_t phaseC = 0;

void BSP_ADC_Start(void)
{
    HAL_ADCEx_InjectedStart_IT(&hadc1);
}

uint16_t BSP_ADC_GetPhaseA(void)
{
    return phaseA;
}

uint16_t BSP_ADC_GetPhaseB(void)
{
    return phaseB;
}

uint16_t BSP_ADC_GetPhaseC(void)
{
    return phaseC;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if(hadc->Instance != ADC1)
        return;

    phaseA = HAL_ADCEx_InjectedGetValue(
                hadc,
                ADC_INJECTED_RANK_1);

    phaseB = HAL_ADCEx_InjectedGetValue(
                hadc,
                ADC_INJECTED_RANK_2);

    phaseC = HAL_ADCEx_InjectedGetValue(
                hadc,
                ADC_INJECTED_RANK_3);
}