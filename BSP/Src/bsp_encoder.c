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
#include <stdint.h>

extern SPI_HandleTypeDef hspi1;

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

    /* RX FIFO 门限 = 1 字节 (与 HAL 的 8-bit 接收路径一致),
     * 这样寄存器级逐字节读写才能每字节都等到 RXNE */
    SET_BIT(hspi1.Instance->CR2, SPI_RXFIFO_THRESHOLD);
}

/* ===================================================================
 * 读取原始角度 (寄存器级, 不依赖 HAL)
 *
 * 在 1kHz FOC 中断里调用, 所以:
 *   1) 不能用 HAL_SPI_TransmitReceive(HAL_MAX_DELAY) —— 它内部会加锁,
 *      查状态机, 并且用 HAL_GetTick() 做超时; SPI 一旦异常会死等,
 *      把整个 FOC 中断挂死 (PWM 停在旧占空比)。
 *   2) 所有等待都带自旋上限, 超时就返回上一次有效值, 中断永不死等。
 * =================================================================== */

#define ENC_SPI_GUARD   3000u   /* 单次等待最大自旋次数 (~18us @170MHz) */

static uint16_t enc_last_raw = 0;   /* 上一次有效读数 (超时兜底) */

uint16_t BSP_Encoder_ReadAngle(void)
{
    SPI_TypeDef *spi = hspi1.Instance;
    uint32_t     guard;

    /* 保证门限为 1 字节 (第一次调用时 Init 已设置, 这里幂等重设) */
    SET_BIT(spi->CR2, SPI_RXFIFO_THRESHOLD);

    /* 溢出恢复: 读 DR 再读 SR 可清 OVR 标志, 避免接收卡死 */
    if (spi->SR & SPI_SR_OVR)
    {
        (void)spi->DR;
        (void)spi->SR;
    }

    const uint8_t tx[2] =
    {
        (uint8_t)(KTH7823_CMD_READ_ANGLE >> 8),
        (uint8_t)(KTH7823_CMD_READ_ANGLE & 0xFF)
    };
    uint8_t rx[2] = { 0u, 0u };

    /* 确保 SPI 使能: 不再走 HAL, 必须自己维护 SPE
     * (HAL_SPI_TransmitReceive 在部分路径结束时会关掉 SPE) */
    if ((spi->CR1 & SPI_CR1_SPE) == 0u)
    {
        SET_BIT(spi->CR1, SPI_CR1_SPE);
    }

    ENC_CS_LOW();

    for (uint32_t i = 0u; i < 2u; i++)
    {
        guard = ENC_SPI_GUARD;
        while ((spi->SR & SPI_SR_TXE) == 0u)
        {
            if (--guard == 0u) { ENC_CS_HIGH(); return enc_last_raw; }
        }
        *(__IO uint8_t *)&spi->DR = tx[i];

        guard = ENC_SPI_GUARD;
        while ((spi->SR & SPI_SR_RXNE) == 0u)
        {
            if (--guard == 0u) { ENC_CS_HIGH(); return enc_last_raw; }
        }
        rx[i] = *(__IO uint8_t *)&spi->DR;
    }

    /* 等最后一拍 SCK 结束 (手册要求 CS 上升前 SCK 高电平 ≥120ns) */
    guard = ENC_SPI_GUARD;
    while (spi->SR & SPI_SR_BSY)
    {
        if (--guard == 0u) break;
    }

    ENC_CS_HIGH();

    /* 拼回 16-bit */
    uint16_t raw = ((uint16_t)rx[0] << 8) | (uint16_t)rx[1];
    raw &= KTH7823_ANGLE_MASK;

    enc_last_raw = raw;

    /* 返回 16-bit 绝对角度 (0~65535 → 0°~360°) */
    return raw;
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

