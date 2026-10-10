#include "inv_park.h"

/* 注意: 传入的是 theta 的 sin/cos (与 Park_Run 共用同一组, 见 park.c) */
void InvPark_Run(const DQ_t *in,
                 float s,
                 float c,
                 AlphaBeta_t *out)
{
    out->alpha =
        in->d * c -
        in->q * s;

    out->beta =
        in->d * s +
        in->q * c;
}