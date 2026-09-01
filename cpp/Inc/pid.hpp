#pragma once

namespace foc {

class PI{
    public:
    PI() = default;

    PI(float kp, float ki, float out_min, float out_max)
        : kp_(kp), ki_(ki), int_limit_(out_max),
          out_max_(out_max), out_min_(out_min) {}

        float run(float ref, float feedback, float dt);

        void setGains( float kp, float ki);

        void setIntLimit(float limit) { int_limit_ = limit;}
        void reset() { integral_ = 0.0f; }

        float integral() const {return integral_;}

        float kp() const {return kp_;}

        float ki() const {return ki_;}

    private:
        float kp_        = 0.0f;   // 注意 C++11 起的"就地初始化"，构造里不写也有默认值
        float ki_        = 0.0f;
        float integral_  = 0.0f;
        float int_limit_ = 0.0f;
        float out_max_   = 0.0f;
        float out_min_   = 0.0f;
};    

} // namespace foc
