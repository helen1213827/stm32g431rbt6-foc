/**
 * @file    bsp_vofa.c
 * @brief   VOFA+ JustFloat 协议驱动实现
 *
 * 协议细节:
 *   - 每通道 4 字节 float32, 小端字节序
 *   - 帧尾标记: 0x00 0x00 0x80 0x7F (float +Inf)
 *   - 使用阻塞式 HAL_UART_Transmit, 适合 1~10kHz 级别的低延迟发送
 *
 * 带宽估算 (115200bps):
 *   单帧 = channels*4 + 4 字节
 *   6 通道 → 28 字节/帧 → 最大 ~410 帧/秒
 *   建议在 FOC 循环中降频发送 (如每 10 次发一次)
 *
 * 更高频率建议: 在 CubeMX 中把 USART3 波特率改为 921600 或 2M
 */

#include "bsp_vofa.h"
#include "usart.h"
#include <string.h>

extern UART_HandleTypeDef huart3;

/* 帧缓冲: 最大通道数 * 4 + 4 字节 tail */
static uint8_t  vofa_buf[VOFA_MAX_CHANNELS * 4 + 4];
static uint8_t  vofa_tail[4] = { 0x00, 0x00, 0x80, 0x7F };

/* VOFA 波形模式开关: 1=发 JustFloat 帧(主循环), 0=文本打印 */
uint8_t g_vofa = 0;

/**
 * @brief 初始化 VOFA
 */
void BSP_VOFA_Init(void)
{
    /* USART3 已在 CubeMX MX_USART3_UART_Init() 中初始化 */
    /* 此处预留, 无额外操作 */
}

/**
 * @brief 发送一帧 float 数据
 *
 * 将 float 数组按小端字节序打包, 追加帧尾, 通过 USART3 发出.
 * 使用阻塞发送以保证数据完整性.
 */
void BSP_VOFA_Send(const float *data, uint8_t channels)
{
    if (channels == 0 || channels > VOFA_MAX_CHANNELS) return;
    if (data == NULL) return;

    /* 打包: 每个 float → 4 字节小端 */
    uint8_t *p = vofa_buf;
    for (uint8_t i = 0; i < channels; i++)
    {
        uint32_t raw;
        memcpy(&raw, &data[i], sizeof(float));
        p[0] = (uint8_t)(raw);
        p[1] = (uint8_t)(raw >> 8);
        p[2] = (uint8_t)(raw >> 16);
        p[3] = (uint8_t)(raw >> 24);
        p += 4;
    }

    /* 追加帧尾 */
    memcpy(p, vofa_tail, 4);
    p += 4;

    /* 发送 */
    uint16_t len = p - vofa_buf;
    HAL_UART_Transmit(&huart3, vofa_buf, len, HAL_MAX_DELAY);
}
