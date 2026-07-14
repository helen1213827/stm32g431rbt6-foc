#include "current.h"
#include "bsp_adc.h"
#include <stdint.h>

Current_t current ;

void Current_Init(void)
{

    current.offset_a = 0;
    current.offset_b = 0;
    current.offset_c = 0;

    current.ia = 0.0f;
    current.ib = 0.0f;
    current.ic = 0.0f;
}

void Current_CalibrateOffset(void)
{
    uint32_t sumA = 0;
    uint32_t sumB = 0;
    uint32_t sumC = 0;
    uint16_t i;

    for (i = 0; i < 1000; i++)
    {
        sumA += BSP_ADC_GetPhaseA();
        sumB += BSP_ADC_GetPhaseB();
        sumC += BSP_ADC_GetPhaseC();

        HAL_Delay(1);
    }

    current.offset_a = sumA / 1000;
    current.offset_b = sumB / 1000;
    current.offset_c = sumC / 1000;
}

void Current_Update(void)
{
    current.raw_a = BSP_ADC_GetPhaseA();
    current.raw_b = BSP_ADC_GetPhaseB();
    current.raw_c = BSP_ADC_GetPhaseC();

    current.ia =
        ((float)current.raw_a - current.offset_a)
        * CURRENT_SCALE;

    current.ib =
        ((float)current.raw_b - current.offset_b)
        * CURRENT_SCALE;

    current.ic =
        ((float)current.raw_c - current.offset_c)
        * CURRENT_SCALE;
}