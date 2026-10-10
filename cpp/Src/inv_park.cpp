#include "inv_park.hpp"

#include <cmath>


namespace foc {
    AlphaBeta invPark (const DQ& in, float theta){
        const float s = std::sin(theta);
        const float c = std::cos(theta);
        return AlphaBeta{
            in.d * c - in.q * s,
            in.d * s + in.q * c
        };
    }
}



