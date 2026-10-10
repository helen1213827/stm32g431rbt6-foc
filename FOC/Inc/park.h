#ifndef PARK_H
#define PARK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "math_types.h"

/* s / c 为 theta 的 sin/cos (Park 与 InvPark 共算一次) */
void Park_Run(const AlphaBeta_t *in,
              float s,
              float c,
              DQ_t *out);

#ifdef __cplusplus
}
#endif

#endif
