#ifndef FOC_H
#define FOC_H


#include "current_loop.h"
#include "speed_loop.h"
#include "position_loop.h"


#ifdef __cplusplus
extern "C" {
#endif


#define FOC_DT      (0.001f)
#define VBUS        (12.0f)
#define VBUS_HALF   (VBUS / 2.0f)


typedef enum
{
    FOC_MODE_DISABLED = 0,
    FOC_MODE_ALIGN,
    FOC_MODE_VOLATAGE_OPENLOOP,
    FOC_MODE_CURRENT,
    FOC_MODE_SPEED,
    FOC_MODE_POSITION,
} FOC_Mode_t;

extern FOC_Mode_t g_foc_mode;
extern float g_vd_open;
extern float g_vq_open;
extern float g_id_ref;
extern float g_iq_ref;
extern float g_off;          /* 校准后的电零点偏移(rad), debugger 可实时微调 */
extern float g_align_vd;     /* 开环对齐时的 d 轴电压 (V) */
extern uint16_t g_enc_cache; /* FOC 循环缓存的编码器原始值 (供打印, 避免 SPI 冲突) */

/* 电流测量坐标补偿矩阵 (静止系, true = C·measured)
 * ALIGN 双探针自动标定, 修正相位映射不一致(旋转/镜像/增益差) */
typedef struct {
    float c00, c01;
    float c10, c11;
} CurFrame_t;
extern CurFrame_t g_cur_frame;
extern float g_iq_sign;      /* q 轴电流反馈符号 (±1), debugger 可调 */
extern float g_spd_sign;     /* 速度环输出符号 (±1), debugger 可调 */

/* 标定出的坐标旋转角 (供打印/观测, 单位 rad) */
float FOC_GetCurRot(void);
float FOC_GetPosSpeedRef(void);

/* PI 增益在线调节 (串口命令用) */
void FOC_SetCurrentGains(float kp, float ki);
void FOC_SetSpeedGains(float kp, float ki);
void FOC_SetPositionGains(float kp, float ki);
void FOC_SetPositionDamping(float kd);
void FOC_PositionReset(void);

extern float g_speed_ref;
extern float g_pos_ref;
extern float g_iq_ref_cmd;
extern float g_sl_iq_ref_out;


void FOC_Init(void);

void FOC_Run(void);

void FOC_SetMode(FOC_Mode_t mode);


FOC_Mode_t FOC_GetMode(void);

#ifdef __cplusplus
}
#endif

#endif