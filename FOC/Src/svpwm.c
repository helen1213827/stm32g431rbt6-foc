#include "svpwm.h"

static float max3(float a, float b, float c)
{
    float m = a;

    if (b > m)
        m = b;

    if (c > m)
        m = c;

    return m;
}

static float min3(float a, float b, float c)
{
    float m = a;

    if (b < m)
        m = b;

    if (c < m)
        m = c;

    return m;
}

void SVPWM_Run(float alpha,
               float beta,
               SVPWM_t *out)
{
    float ua;
    float ub;
    float uc;

    ua = alpha;

    ub = -0.5f * alpha
       + 0.8660254f * beta;

    uc = -0.5f * alpha
       - 0.8660254f * beta;

    float vmax = max3(ua, ub, uc);
    float vmin = min3(ua, ub, uc);

    float offset = (vmax + vmin) * 0.5f;

    ua -= offset;
    ub -= offset;
    uc -= offset;

    out->duty_a = ua * 0.5f + 0.5f;
    out->duty_b = ub * 0.5f + 0.5f;
    out->duty_c = uc * 0.5f + 0.5f;

    if(out->duty_a > 1.0f) out->duty_a = 1.0f;
    if(out->duty_a < 0.0f) out->duty_a = 0.0f;

    if(out->duty_b > 1.0f) out->duty_b = 1.0f;
    if(out->duty_b < 0.0f) out->duty_b = 0.0f;

    if(out->duty_c > 1.0f) out->duty_c = 1.0f;
    if(out->duty_c < 0.0f) out->duty_c = 0.0f;
}
