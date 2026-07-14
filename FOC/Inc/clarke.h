#ifndef CLARKE_H
#define CLARKE_H
#include "math_types.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float alpha;
    float beta;

}Clarke_t;
void Clarke_Run(const ABC_t *in,
                AlphaBeta_t *out);

#ifdef __cplusplus
}
#endif

#endif