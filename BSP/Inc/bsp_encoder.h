/**
 * @file    bsp_encoder.h
 * @brief   KTH7823 磁编码器 SPI 驱动
 *
 * KTH7823 是一款 16-bit 霍尔磁编码器，通过 SPI 接口读取绝对角度。
 * 参考数据手册: 数据手册/kth7823.pdf
 *
 * 硬件连接 (SPI1):
 *   PB3  → SCK
 *   PB4  → MISO
 *   PB5  → MOSI
 *   PD2  → CS  (软件控制)
 *
 * SPI 参数（依据手册第 5 章）:
 *   Mode 3 (CPOL=1, CPHA=1), 8-bit 帧, MSB First
 *   SCK 周期 ≥ 100ns → 推荐 ≤ 10MHz
 *   数据帧: 固定 16-bit
 */

#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ===================================================================
 * KTH7823 协议定义（依据数据手册 5.4 节）
 * ===================================================================
 * 读角度: 发任意指令（建议 MOSI=0x0000 避免误触寄存器操作）
 *         返回 16-bit 绝对角度数据，MSB 在前
 * 角度范围: 0 ~ 65535 对应 0° ~ 360°
 * 换算: angle_rad = raw / 65536 * 2π
 */

/** 读取角度的 SPI 命令（保持 MOSI 低电平，防止误触发寄存器读写） */
#define KTH7823_CMD_READ_ANGLE   0x0000U

/** 角度数据掩码 (16-bit) */
#define KTH7823_ANGLE_MASK       0xFFFFU

/** 16-bit 满量程 */
#define KTH7823_ANGLE_MAX        65536U

/* ===================================================================
 * API
 * =================================================================== */

/**
 * @brief 初始化编码器（CS 引脚）
 */
void BSP_Encoder_Init(void);

/**
 * @brief 读取原始 16-bit 角度值
 * @return 0~65535 的原始角度值 (0°~360°)
 */
uint16_t BSP_Encoder_ReadAngle(void);

/**
 * @brief 读取角度并转换为弧度
 * @return 角度 [0, 2π) rad
 */
float BSP_Encoder_GetAngleRad(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ENCODER_H */
