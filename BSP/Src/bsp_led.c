#include "bsp_led.h"
#include "main.h"

/* 心跳灯：每 500ms 翻转一次（在 main while 中配合 HAL_Delay 调用） */
static uint32_t heartbeat_tick = 0;

/* FOC 循环指示：每 250 次 FOC 翻转一次 → 2Hz @ 1kHz FOC */
#define FOC_LED_DIVIDER 250
static uint16_t foc_tick_count = 0;

void BSP_LED_Init(void)
{
    /* GPIO 初始化由 CubeMX 生成的 MX_GPIO_Init 完成 */
    /* 这里只确保初始状态 */
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
}

/* ---- 心跳灯：每次调用计数，500ms 翻转 ---- */
void BSP_LED_Heartbeat(void)
{
    heartbeat_tick++;

    if (heartbeat_tick >= 500)
    {
        heartbeat_tick = 0;
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }
}

/* ---- FOC 循环指示：每 250 次翻转 ---- */
void BSP_LED_FocTick(void)
{
    foc_tick_count++;

    if (foc_tick_count >= FOC_LED_DIVIDER)
    {
        foc_tick_count = 0;
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    }
}

/* ---- 故障灯 ---- */
void BSP_LED_FaultOn(void)
{
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
}

void BSP_LED_FaultOff(void)
{
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
}


