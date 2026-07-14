#include "foc.h"
#include "clarke.h"
#include "park.h"
#include "motor.h"

static AlphaBeta_t i_ab;
static DQ_t i_dq;

void FOC_Run(void)
{
    ABC_t abc =
    {
        .a = motor.ia,
        .b = motor.ib,
        .c = motor.ic
    };

    Clarke_Run(&abc, &i_ab);

    Park_Run(&i_ab, motor.theta, &i_dq);

    motor.ialpha = i_ab.alpha;
    motor.ibeta = i_ab.beta;

    motor.id = i_dq.d;
    motor.iq = i_dq.q;
}