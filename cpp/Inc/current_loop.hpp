#pragma once 
#include "pid.hpp"

namespace foc {
    class CurrentLoop {
        public:
        CurrentLoop ( float kp, float ki, float out_limit);

        void run(float id_ref, float iq_ref, float id_fb, float iq_fb, float dt);

        void setGains(float d_kp, float d_ki, float q_kp, float q_kli);

        void reset();

        float vd() const {return vd_;}
        float vq() const {return vq_;}
        void setEnabled(bool e) {enabled_ = e;}

        private:
        PI pi_d_;
        PI pi_q_;
        float vd_ = 0.0f;
        float vq_ = 0.0f;
        bool enabled_ = true;
    };
}