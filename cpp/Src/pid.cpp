#include "pid.hpp"

namespace foc {
    float PI::run(float ref, float feedback, float dt) {
        float error = ref - feedback;
        integral_ += error * dt;

        // Clamp the integral term to prevent windup
        if (integral_ > int_limit_) {
            integral_ = int_limit_;
        } else if (integral_ < -int_limit_) {
            integral_ = -int_limit_;
        }

        float output = kp_ * error + ki_ * integral_;

        // Clamp the output to the specified limits
        if (output > out_max_) {
            output = out_max_;
        } else if (output < out_min_) {
            output = out_min_;
        }

        return output;
    }

    void PI::setGains(float kp, float ki) {
        kp_ = kp;
        ki_ = ki;
    }
}