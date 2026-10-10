#include "svpwm.hpp"

namespace foc {
    namespace {
        float max3 (float a, float b, float c)
        {
            float m = a;
            if (b > m) m = b;
            if (c > m) m = c;
            return m;
        }

        float min3 (float a, float b, float c)
        {
            float m = a;
            if (b < m) m = b;
            if (c < m) m = c;
            return m;
        }

        constexpr float kDutyMax = 1.0f;
        constexpr float kDutyMin = 0.0f;


    }

    SVPWM svpwm(float alpha, float beta){
        float ua = alpha;
        float ub = -0.5f * alpha + 0.86602540378f * beta; // sqrt(3)/2 = 0.86602540378
        float uc = -0.5f * alpha - 0.86602540378f * beta;

        const float vmax = max3(ua, ub, uc);
        const float vmin = min3(ua, ub, uc);
        const float vavg = (vmax + vmin) * 0.5f;

        ua -= vavg;
        ub -= vavg;
        uc -= vavg;

        SVPWM out { ua* 0.5f + 0.5f, ub* 0.5f + 0.5f, uc* 0.5f + 0.5f };

        if (out.duty_a > kDutyMax) out.duty_a = kDutyMax;
        if (out.duty_a < kDutyMin) out.duty_a = kDutyMin;
        if (out.duty_b > kDutyMax) out.duty_b = kDutyMax;
        if (out.duty_b < kDutyMin) out.duty_b = kDutyMin;
        if (out.duty_c > kDutyMax) out.duty_c = kDutyMax;
        if (out.duty_c < kDutyMin) out.duty_c = kDutyMin;

        return out;
    }
}