#ifndef INV_PARK_H
#define INV_PARK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "math_types.h"

/* s / c 为 theta 的 sin/cos (与 Park_Run 共用同一组) */
void InvPark_Run(const DQ_t *in,
                 float s,
                 float c,
                 AlphaBeta_t *out);

#ifdef __cplusplus
}
#endif

#endif