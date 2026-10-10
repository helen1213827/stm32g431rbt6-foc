#include "park.h"

/* 注意: 传入的是 theta 的 sin/cos, 不是 theta。
 * Park 与 InvPark 用同一个 theta, 由调用方算一次再共享 —— 避免
 * 同一周期里重复调用超越函数 (FOC 中断里最贵的一块)。 */
void Park_Run(const AlphaBeta_t *in,
              float s,
              float c,
              DQ_t *out)
{
    out->d =
        in->alpha * c +
        in->beta  * s;

    out->q =
       -in->alpha * s +
        in->beta  * c;
}