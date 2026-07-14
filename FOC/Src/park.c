#include "park.h"
#include <math.h>

void Park_Run(const AlphaBeta_t *in,
              float theta,
              DQ_t *out)
{
    float s = sinf(theta);
    float c = cosf(theta);

    out->d =
        in->alpha * c +
        in->beta  * s;

    out->q =
       -in->alpha * s +
        in->beta  * c;
}