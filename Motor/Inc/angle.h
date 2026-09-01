#ifndef ANGLE_H
#define ANGLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 角度/位置估计模块
 *
 * 支持两种模式:
 *   ANGLE_MODE_OPENLOOP — 开环积分 (给定电角速度)
 *   ANGLE_MODE_ENCODER  — KTH7823 磁编码器 (SPI1) 实时读取
 */

typedef enum {
    ANGLE_MODE_OPENLOOP = 0,
    ANGLE_MODE_ENCODER  = 1
} AngleMode_t;

void Angle_Init(void);

/**
 * @brief 切换角度来源模式
 */
void Angle_SetMode(AngleMode_t mode);

/**
 * @brief 每次 FOC 循环调用，用机械角更新电角度
 *
 * 编码器值由调用方读取后传入（避免 SPI 多线程访问冲突）.
 * @param mech 机械角 (rad)
 * @param dt 控制周期（秒），编码器模式下未使用
 */
void Angle_Update(float mech, float dt);

/**
 * @brief 设置电零点偏移
 * @param offset 偏移量 (rad)
 */
void Angle_SetOffset(float offset);

/**
 * @brief 将当前编码器位置标定为电零点
 */
void Angle_ZeroCurrent(void);

/**
 * @brief 用当前机械角计算电零点偏移 (取模 2π/P)
 *
 * 在对齐模式下电机吸附到 theta=0 后调用,
 * 使 theta = -(mech - offset)*P 在吸附位置恰好为 0.
 * @return 校准后的偏移量 (rad)
 */
float Angle_CalibrateOffset(void);

/**
 * @brief 设置目标电角速度（开环备用）
 * @param omega_elec 电角速度 (rad/s)
 */
void Angle_SetSpeed(float omega_elec);

/**
 * @brief 获取当前电角度
 * @return 电角度 (rad), [0, 2π)
 */
float Angle_GetTheta(void);

/**
 * @brief 获取当前编码器偏移量
 * @return 机械角度偏移 (rad)
 */
float Angle_GetOffset(void);

#ifdef __cplusplus
}
#endif

#endif // ANGLE_H
