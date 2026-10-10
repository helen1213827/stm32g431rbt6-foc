#include "bsp_adc.h"

extern ADC_HandleTypeDef hadc1;

void BSP_ADC_Start(void)
{
    HAL_ADCEx_InjectedStart_IT(&hadc1);

    /* 注入组由 TIM1_TRGO2 硬件触发: JADSTART 置一次后随每个触发沿自动重装,
     * 结果我们在 FOC 中断里轮询 JDRx 读取, 全程不需要 EOC 中断。
     * 关掉 NVIC 可直接省掉 20kHz(= PWM 频率) × HAL_ADC_IRQHandler 的开销,
     * 也避免它和 FOC 中断抢优先级。(注入转换本身不受影响) */
    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
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

/* 注入组由 T1_TRGO2 外部触发: JADSTART 置一次后硬件随每个触发沿自动重装,
 * HAL 在这种模式下不会重新武装/不需 ISR, 标志位也不门控转换。
 * 这里只是把 ISR 原本干的"清标志"搬到 FOC 循环, 保证行为与旧版一致。 */
void BSP_ADC_ClearFlags(void)
{
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
}