#include "bsp_pwm.h"



extern TIM_HandleTypeDef htim1;

void BSP_PWM_Init(void)
{

}

void BSP_PWM_Start(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void BSP_PWM_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

void BSP_PWM_SetDuty(float dutyA,float dutyB, float dutyC)
{
    if(dutyA < 0.0f) dutyA = 0.0f;
    if(dutyA > 1.0f) dutyA = 1.0f;
    if(dutyB < 0.0f) dutyB = 0.0f;
    if(dutyB > 1.0f) dutyB = 1.0f;
    if(dutyC < 0.0f) dutyC = 0.0f;
    if(dutyC > 1.0f) dutyC = 1.0f;

    uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(arr * dutyA));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)(arr * dutyB));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint16_t)(arr * dutyC));
}