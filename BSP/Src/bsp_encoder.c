/**
 * @file    bsp_encoder.c
 * @brief   KTH7823 磁编码器 SPI 驱动实现
 *
 * 协议说明（依据数据手册第 5 章）:
 *   - SPI Mode 3: CPOL=1 (空闲高), CPHA=1 (第二边沿采样)
 *   - 数据帧固定 16-bit，分 2 个字节传输
 *   - 读角度: CS↓ → 发 0x0000 → 收 16-bit 角度 → CS↑
 *   - MOSI 保持低电平避免误触发寄存器操作 (opcode=01 读, 10 写)
 *   - SCK 周期 ≥ 100ns, CS↑ 前 SCK↑ 后需 ≥ 120ns
 *
 * 寄存器操作（备用）:
 *   读寄存器: opcode=01, addr[5:0], 8'b0
 *   写寄存器: opcode=10, addr[5:0], data[7:0]  (帧间需等待 ≥20ms)
 */

#include "bsp_encoder.h"
#include "spi.h"

/* ===================================================================
 * CS 引脚宏
 * =================================================================== */

#define ENC_CS_LOW()   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET)
#define ENC_CS_HIGH()  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET)

/* ===================================================================
 * 初始化
 * =================================================================== */

void BSP_Encoder_Init(void)
{
    /*
     * SPI1 由 CubeMX 初始化（见下方 CubeMX 配置要求）
     * 这里只做 CS 引脚初始化
     */
    ENC_CS_HIGH();
}

/* ===================================================================
 * 读取原始角度
 * =================================================================== */

uint16_t BSP_Encoder_ReadAngle(void)
{
    /*
     * 16-bit SPI 帧 = 2 字节
     * 因为 STM32 是小端，我们用数组保证字节顺序:
     *   tx[0] = 命令高字节
     *   tx[1] = 命令低字节
     */
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = (uint8_t)(KTH7823_CMD_READ_ANGLE >> 8);
    tx[1] = (uint8_t)(KTH7823_CMD_READ_ANGLE & 0xFF);

    /* CS 拉低 → 传输 → CS 拉高 */
    ENC_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    ENC_CS_HIGH();

    /* 拼回 16-bit */
    uint16_t raw = ((uint16_t)rx[0] << 8) | (uint16_t)rx[1];

    /* 返回 16-bit 绝对角度 (0~65535 → 0°~360°) */
    return raw & KTH7823_ANGLE_MASK;
}

/* ===================================================================
 * 读取角度 (弧度)
 * =================================================================== */

float BSP_Encoder_GetAngleRad(void)
{
    uint16_t raw = BSP_Encoder_ReadAngle();
    return BSP_Encoder_CountToRad(raw);
}

float BSP_Encoder_CountToRad(uint16_t raw)
{
    return (float)raw / (float)KTH7823_ANGLE_MAX * 6.283185307179586f;
}

