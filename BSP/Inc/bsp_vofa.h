/**
 * @file    bsp_vofa.h
 * @brief   VOFA+ 上位机通信驱动 (JustFloat 协议)
 *
 * 协议: 每个通道 4 字节 float32 (小端), 帧尾 0x00 0x00 0x80 0x7F
 * 硬件: USART3 (PB10 TX, PB11 RX)
 *
 * VOFA+ 上位机设置:
 *   协议: JustFloat
 *   端口: 对应串口
 *   波特率: 需与 MCU 一致
 *
 * 用法:
 *   float buf[] = { id, iq, speed, theta };
 *   BSP_VOFA_Send(buf, 4);
 */

#ifndef BSP_VOFA_H
#define BSP_VOFA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 单帧最大通道数 */
#define VOFA_MAX_CHANNELS  16

/**
 * @brief 初始化 VOFA 通信
 * @note  USART3 已在 CubeMX 中初始化，此函数预留扩展
 */
void BSP_VOFA_Init(void);

/**
 * @brief 发送一帧 float 数据到 VOFA+
 * @param data    float 数组指针
 * @param channels 通道数 (1 ~ VOFA_MAX_CHANNELS)
 *
 * 帧格式: [ch0_le][ch1_le]...[chN-1_le][tail: 0x00 0x00 0x80 0x7F]
 */
void BSP_VOFA_Send(const float *data, uint8_t channels);

#ifdef __cplusplus
}
#endif

#endif /* BSP_VOFA_H */
