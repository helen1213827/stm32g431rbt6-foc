#include "angle.h"
#include "bsp_encoder.h"
#include <math.h>

#define TWO_PI (2.0f * 3.14159265358979f)
#define POLE_PAIRS  (14)   /* 4310 电机极对数 */

static float theta;            /* 电角度 (rad) */
static float theta_offset;     /* 电零点偏移 (rad) */
static float omega_elec;       /* 电角速度 (rad/s) */
static AngleMode_t mode;       /* 当前模式 */

void Angle_Init(void)
{
    theta        = 0.0f;
    theta_offset = 0.0f;
    omega_elec   = 0.0f;
    mode         = ANGLE_MODE_OPENLOOP;  /* 默认开环 */

    BSP_Encoder_Init();
}

void Angle_SetMode(AngleMode_t m)
{
    mode = m;
}

void Angle_Update(float mech, float dt)
{
    if (mode == ANGLE_MODE_ENCODER)
    {
        /* 编码器模式: 负号因安装反向 */
        theta = -(mech - theta_offset) * POLE_PAIRS;
    }
    else
    {
        theta += omega_elec * dt;

        /* 自动学习偏移 (编码器方向与电气相反) */
        float enc_elec = -mech * POLE_PAIRS;
        float diff = enc_elec - theta;
        while (diff >  3.14159265358979f) diff -= TWO_PI;
        while (diff < -3.14159265358979f) diff += TWO_PI;

        /* 低通滤波收敛到正确偏移 */
        theta_offset += 0.005f * (diff / POLE_PAIRS - theta_offset);
    }

    /* 归一化到 [0, 2π) */
    while (theta >= TWO_PI)
        theta -= TWO_PI;

    while (theta < 0.0f)
        theta += TWO_PI;
}

/**
 * @brief 设置电零点偏移（校准用）
 * @param offset 偏移量 (rad)
 */
void Angle_SetOffset(float offset)
{
    theta_offset = offset;
}

/**
 * @brief 将当前位置设为零点
 */
void Angle_ZeroCurrent(void)
{
    theta_offset = BSP_Encoder_GetAngleRad();
}

/**
 * @brief 用当前机械角计算电零点偏移 (取模 2π/P)
 *
 * 让 theta = -(mech - offset)*P 在吸附位置恰好为 0:
 *   offset ≡ mech (mod 2π/P)
 * 取模后消除 180° 歧义 (id>0 方向唯一).
 */
float Angle_CalibrateOffset(void)
{
    float mech   = BSP_Encoder_GetAngleRad();
    float period = TWO_PI / POLE_PAIRS;

    theta_offset = fmodf(mech, period);
    if (theta_offset < 0.0f)
        theta_offset += period;

    return theta_offset;
}

void Angle_SetSpeed(float omega)
{
    omega_elec = omega;
}

float Angle_GetTheta(void)
{
    return theta;
}

float Angle_GetOffset(void)
{
    return theta_offset;
}

float Angle_MechToElecSpeed(float mech_speed)
{
    return mech_speed * (float)POLE_PAIRS;
}
