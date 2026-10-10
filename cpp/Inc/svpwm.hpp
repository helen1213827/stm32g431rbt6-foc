#pragma once 

namespace foc {
    struct SVPWM {float duty_a = 0.0f, duty_b  = 0.0f, duty_c = 0.0f;};

    SVPWM svpwm(float alpha, float beta);
}