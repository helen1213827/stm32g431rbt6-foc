#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32g4xx_hal.h"

#define CAN_ID_MOTOR_STATE1  0x101
#define CAN_ID_MOTOR_STATE2  0x102

#define CAN_ID_CONTROL_CMD     0x201

#define CAN_SCALE_CURRENT  1000.0f
#define CAN_SCALE_VELOCITY  100.0f
#define CAN_SCALE_SPEED  100.0f



/* ================================================================
 * API 函数声明
 * ================================================================ */

/**
 * @brief 初始化 CAN 通信
 *
 * 具体做了:
 *   1. 配置接收过滤器 (只收 CAN_ID_CONTROL_CMD)
 *   2. 使能 RX FIFO0 新消息中断
 *   3. 启动 FDCAN1
 *
 * @note 必须在 MX_FDCAN1_Init() 之后调用
 */
void BSP_CAN_Init(void);

/**
 * @brief 发送一条 CAN 消息 (通用接口)
 *
 * @param id    11-bit 标准帧 ID
 * @param data  数据指针 (最多 8 字节)
 * @param len   数据长度 (0~8)
 *
 * @note 放入 TX FIFO, 硬件自动发送, 非阻塞
 */
void BSP_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len);

/**
 * @brief 发送浮点数组 (自动按 SCALE 转换为 int16)
 *
 * @param id      CAN ID
 * @param values  float 数组
 * @param count   float 数量 (最多 4 个, 因为 4×2=8 字节)
 * @param scale   缩放因子 (如 CAN_SCALE_CURRENT)
 */
void BSP_CAN_SendFloats(uint32_t id, const float *values,
                        uint8_t count, float scale);

/**
 * @brief 快捷发送: 发送电机状态 (两个 ID 各一帧)
 *
 * 发送 0x101: speed, ia, ib, ic
 * 发送 0x102: id, iq, vd, vq
 */
void BSP_CAN_SendMotorState(void);


/* ================================================================
 * 以下由 HAL 中断自动调用, 你不需要手动调
 * ================================================================ */

/**
 * @brief FDCAN1 RX FIFO0 接收回调 (HAL 弱函数重写)
 *
 * 当收到匹配过滤器的消息时, HAL 中断会调用此函数.
 * 在这里解析控制命令, 更新 motor 全局变量.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0_ITs);



#ifdef __cplusplus
}
#endif


#endif