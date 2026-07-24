/**
 * @file    uart_printf.c
 * @brief   printf 重定向到 USART3 (PB10 TX, PB11 RX, 115200-8-N-1)
 *
 * 用法: 调用 printf("Hello %d\n", val); 即可从串口输出
 */

#include "usart.h"
#include <stdio.h>

extern UART_HandleTypeDef huart3;

/**
 * @brief 重载 newlib _write, 将 stdout 定向到 USART3
 */
int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
