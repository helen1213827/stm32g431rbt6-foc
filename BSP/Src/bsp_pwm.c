#include "bsp_pwm.h"

extern TIM_HandleTypeDef htim1;

#define PWM_DUTY_MAX 0.95f
#define PWM_DUTY_MIN 0.05f

void BSP_PWM_Init(void) { }

void BSP_PWM_Start(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);  /* CH4 触发 ADC 注入转换 */
}

void BSP_PWM_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_4);
}

void BSP_PWM_SetDuty(float dutyA, float dutyB, float dutyC)
{
    if (dutyA < PWM_DUTY_MIN) dutyA = PWM_DUTY_MIN;
    if (dutyA > PWM_DUTY_MAX) dutyA = PWM_DUTY_MAX;
    if (dutyB < PWM_DUTY_MIN) dutyB = PWM_DUTY_MIN;
    if (dutyB > PWM_DUTY_MAX) dutyB = PWM_DUTY_MAX;
    if (dutyC < PWM_DUTY_MIN) dutyC = PWM_DUTY_MIN;
    if (dutyC > PWM_DUTY_MAX) dutyC = PWM_DUTY_MAX;

    uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(arr * dutyA));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)(arr * dutyB));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint16_t)(arr * dutyC));
}