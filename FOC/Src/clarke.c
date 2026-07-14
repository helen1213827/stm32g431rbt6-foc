#include "clarke.h"

#define INV_SQRT3 0.57735026919f

void Clarke_Run(const ABC_t *in,
                AlphaBeta_t *out)
{
    out->alpha = in->a;

    out->beta =
        (in->a + 2.0f * in->b) * INV_SQRT3;
}