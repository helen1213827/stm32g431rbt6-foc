#include "foc.h"
#include "bsp_encoder.h"
#include "clarke.h"
#include "math_types.h"
#include "park.h"
#include "motor.h"
#include "current.h"
#include "inv_park.h"
#include "svpwm.h"
#include "bsp_pwm.h"
#include "pid.h"
#include "angle.h"
#include "bsp_led.h"
#include <stdio.h>
#include <math.h>

#define FOC_DT      (0.001f)
#define VBUS        (12.0f)
#define VBUS_HALF   (VBUS / 2.0f)

AlphaBeta_t i_ab, v_ab;
DQ_t        i_dq;
SVPWM_t     pwm;
PI_t        pi_d, pi_q, pi_speed, pi_pos;
static uint16_t last_enc = 0;
float g_speed_ref = -10.0f;
float g_pos_ref = 30000.0f;       /* 目标编码器位置, debug 可改 */       /* 目标速度 rad/s */

float g_iq_ref = -0.3f;

/* DC 对齐状态机 */
typedef enum { S_ALIGN, S_RUN } State_t;
static State_t  state = S_ALIGN;
static uint32_t align_cnt = 0;
#define ALIGN_TIME  3000


void FOC_Init(void)
{
    PI_Init(&pi_d, 0.5f, 5.0f, -VBUS_HALF, VBUS_HALF);
    PI_Init(&pi_q, 0.5f, 5.0f, -VBUS_HALF, VBUS_HALF);
    PI_Init(&pi_speed, 0.03f, 0.01f, -5.0f, 5.0f);
    PI_Init(&pi_pos,   0.05f, 0.0f,  -30.0f, 30.0f);  /* 纯P */
    Angle_Init();
    Current_Init();
}

void FOC_Run(void)
{
    Current_Update();

    if (state == S_ALIGN)
    {
        motor.theta = 0.0f;
        motor.vd = 6.0f;
        motor.vq = 0.0f;

        align_cnt++;
        if (align_cnt == ALIGN_TIME)
        {
            Angle_SetOffset(BSP_Encoder_GetAngleRad() + 0.224f);  /* +π/14 */
            state = S_RUN;
        }
    }
    else
    {
        Angle_Update(FOC_DT);
        motor.theta = Angle_GetTheta();
        motor.ia = current.ia;
        motor.ib = current.ib;
        motor.ic = current.ic;

        ABC_t abc = { .a = motor.ia, .b = motor.ib, .c = motor.ic };
        Clarke_Run(&abc, &i_ab);
        Park_Run(&i_ab, motor.theta, &i_dq);

        motor.id = i_dq.d;
        motor.iq = i_dq.q;

        /* 速度测量 + 低通 */
        uint16_t enc = BSP_Encoder_ReadAngle();
        int32_t  delta = (int32_t)enc - (int32_t)last_enc;
        if      (delta >  32767) delta -= 65536;
        else if (delta < -32768) delta += 65536;
        if (delta >= -3000 && delta <= 3000) last_enc = enc;
        float spd_raw = (float)delta * (6.283185f / 65536.0f) * 1000.0f;
        static float spd_filt = 0.0f;
        spd_filt = spd_filt * 0.95f + spd_raw * 0.05f;
        motor.speed = spd_filt;

        /* 速度环 100Hz */
        static uint32_t spd_cnt = 0;
        spd_cnt++;
        if (spd_cnt >= 10) {
            spd_cnt = 0;

            /* 位置环 */
            float pos_err = g_pos_ref - (float)enc;
            if (pos_err > 32768.0f)  pos_err -= 65536.0f;
            if (pos_err < -32768.0f) pos_err += 65536.0f;
            static float pos_int = 0.0f;
            pos_int += pos_err * 0.01f;
            if      (pos_int >  30.0f) pos_int =  30.0f;
            else if (pos_int < -30.0f) pos_int = -30.0f;
            g_speed_ref = 0.05f * pos_err + 0.001f * pos_int;
            if      (g_speed_ref >  30.0f) g_speed_ref =  30.0f;
            else if (g_speed_ref < -30.0f) g_speed_ref = -30.0f;

            /* 速度环 */
            motor.iq_ref = PI_Run(&pi_speed, g_speed_ref, spd_filt, 0.01f);
        }

        motor.vd = PI_Run(&pi_d, 0.0f, motor.id, FOC_DT);
        motor.vq = PI_Run(&pi_q, motor.iq_ref, motor.iq, FOC_DT);
    }

    DQ_t v_dq = { .d = motor.vd / VBUS_HALF, .q = motor.vq / VBUS_HALF };
    InvPark_Run(&v_dq, motor.theta, &v_ab);

    SVPWM_Run(v_ab.alpha, v_ab.beta, &pwm);
    BSP_PWM_SetDuty(pwm.duty_a, pwm.duty_b, pwm.duty_c);

    BSP_LED_FocTick();
}