#ifndef INV_PARK_H
#define INV_PARK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "math_types.h"

void InvPark_Run(const DQ_t *in,
                 float theta,
                 AlphaBeta_t *out);

#ifdef __cplusplus
}
#endif

#endif