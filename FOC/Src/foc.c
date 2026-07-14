#include "foc.h"
#include "clarke.h"
#include "math_types.h"
#include "park.h"
#include "motor.h"
#include "inv_park.h"
#include "svpwm.h"
#include "bsp_pwm.h"
#include "pid.h"

static AlphaBeta_t i_ab;
static AlphaBeta_t v_ab;
static DQ_t i_dq;
static SVPWM_t pwm;
static PI_t pi_d;
static PI_t pi_q;


void FOC_Init(void)
{
    PI_Init(&pi_d, 2.0f, 0.05f, -12.0f, 12.0f);
    PI_Init(&pi_q, 2.0f, 0.05f, -12.0f, 12.0f);
}
void FOC_Run(void)
{
    ABC_t abc =
    {
        .a = motor.ia,
        .b = motor.ib,
        .c = motor.ic
    };
    DQ_t v_dq =
    {
        .d = motor.vd,
        .q = motor.vq
    };

    Clarke_Run(&abc, &i_ab);

    Park_Run(&i_ab, motor.theta, &i_dq);

    InvPark_Run(&v_dq, motor.theta, &v_ab);


    motor.ialpha = i_ab.alpha;
    motor.ibeta = i_ab.beta;

    motor.id = i_dq.d;
    motor.iq = i_dq.q;

    motor.valpha = v_ab.alpha;
    motor.vbeta = v_ab.beta;

    SVPWM_Run(motor.valpha, motor.vbeta, &pwm);

    motor.dutyA = pwm.duty_a;
    motor.dutyB = pwm.duty_b;
    motor.dutyC = pwm.duty_c;

    BSP_PWM_SetDuty(motor.dutyA, motor.dutyB, motor.dutyC);
}