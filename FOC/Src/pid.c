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

    pi->out_min = out_min;
    pi->out_max = out_max;
}

float PI_Run(PI_t *pi,
             float ref,
             float feedback,
             float dt)
{
    float error;

    float output;

    error = ref - feedback;

    pi->integral += error * pi->ki * dt;

    if(pi->integral > pi->out_max)
        pi->integral = pi->out_max;

    if(pi->integral < pi->out_min)
        pi->integral = pi->out_min;

    output =
        pi->kp * error +
        pi->integral;

    if(output > pi->out_max)
        output = pi->out_max;

    if(output < pi->out_min)
        output = pi->out_min;

    return output;
}