#include "pid.h"

void PI_Init(PI_t *pi,
             float kp,
             float ki,
             float out_min,
             float out_max)
{
    pi->kp = kp;
    pi->ki = ki;

    pi->integral = 0.0f;
    pi->int_limit = out_max;   /* 默认积分限幅=输出限幅 */

    pi->out_min = out_min;
    pi->out_max = out_max;
}

float PI_Run(PI_t *pi,
             float ref,
             float feedback,
             float dt)
{
    float error = ref - feedback;

    /* 条件积分抗饱和: 输出已饱和且误差同向(会把输出推向更饱和)时冻结积分,
     * 防止按住电机/堵转时积分越积越多 → 松手后带着满积分冲过头转圈;
     * 误差反向时允许积分松开. 对电流/速度/位置环通用有效. */
    float out_est = pi->kp * error + pi->integral;
    if (!((out_est > pi->out_max && error > 0.0f) ||
          (out_est < pi->out_min && error < 0.0f)))
    {
        pi->integral += error * pi->ki * dt;
    }

    if(pi->integral >  pi->int_limit)
        pi->integral =  pi->int_limit;
    if(pi->integral < -pi->int_limit)
        pi->integral = -pi->int_limit;

    float output = pi->kp * error + pi->integral;
    if(output > pi->out_max)
        output = pi->out_max;
    if(output < pi->out_min)
        output = pi->out_min;
    return output;
}