#ifndef SVPWM_H
#define SVPWM_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float duty_a;
    float duty_b;
    float duty_c;

}SVPWM_t;

void SVPWM_Run(float alpha,
               float beta,
               SVPWM_t *out);

#ifdef __cplusplus
}
#endif

#endif