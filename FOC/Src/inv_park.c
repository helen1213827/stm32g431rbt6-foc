#include "inv_park.h"
#include <math.h>

void InvPark_Run(const DQ_t *in,
                 float theta,
                 AlphaBeta_t *out)
{
    float s = sinf(theta);
    float c = cosf(theta);

    out->alpha =
        in->d * c -
        in->q * s;

    out->beta =
        in->d * s +
        in->q * c;
}