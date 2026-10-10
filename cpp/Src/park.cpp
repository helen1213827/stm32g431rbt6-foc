#include "park.hpp"
#include <math.h>

namespace foc {
    DQ park (const AlphaBeta& in, float theta)
    {
        float cos_theta = cos(theta);
        float sin_theta = sin(theta);

        return DQ {
            in.alpha * cos_theta + in.beta * sin_theta,
            -in.alpha * sin_theta + in.beta * cos_theta
        };
    }
}