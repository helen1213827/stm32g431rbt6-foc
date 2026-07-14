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
             float feedback);

#ifdef __cplusplus
}
#endif

#endif