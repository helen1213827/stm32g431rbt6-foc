#include "clarke.hpp"

namespace foc {
    namespace {
        constexpr float kInvSqrt3 = 0.5773502691896257f; // 1/sqrt(3)
    }

    AlphaBeta clarke (const ABC& in)
    {
        return AlphaBeta {
            in.a, (in.a+2.0f*in.b)* kInvSqrt3
        };
    }
}