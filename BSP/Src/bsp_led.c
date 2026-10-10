#include "bsp_led.h"
#include "main.h"
#include "foc.h"

/* 心跳灯：每 500ms 翻转一次（在 main while 中配合 HAL_Delay 调用） */
static uint32_t heartbeat_tick = 0;

/* FOC 循环指示：每 250ms 翻转一次 (2Hz)。
 * 由 FOC_FREQ_HZ 派生 —— 改控制频率时不用再动这里。 */
#define FOC_LED_FLIP_MS   250U
#define FOC_LED_DIVIDER   (FOC_LED_FLIP_MS * FOC_FREQ_HZ / 1000U)
static uint32_t foc_tick_count = 0;

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


