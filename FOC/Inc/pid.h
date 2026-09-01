#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp;
    float ki;

    float integral;
    float int_limit;   /* 积分独立限幅 (默认=out_max, 可单独收紧) */

    float out_max;
    float out_min;

}PI_t;

void PI_Init(PI_t *pi,
             float kp,
             float ki,
             float out_min,
             float out_max);

float PI_Run(PI_t *pi,
             float ref,
             float feedback,
             float dt);

#ifdef __cplusplus
}
#endif

#endif