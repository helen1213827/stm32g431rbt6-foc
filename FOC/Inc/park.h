#ifndef PARK_H
#define PARK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "math_types.h"

void Park_Run(const AlphaBeta_t *in,
              float theta,
              DQ_t *out);

#ifdef __cplusplus
}
#endif

#endif
