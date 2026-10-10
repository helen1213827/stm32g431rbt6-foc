current_loop.cpp
/*
 * @Author: helen1213827 2755576035@qq.com
 * @Date: 2026-09-03 10:39:42
 * @LastEditors: helen1213827 2755576035@qq.com
 * @LastEditTime: 2026-09-03 11:33:57
 * @FilePath: /foc/cpp/Src/current_loop.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "current_loop.hpp"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_ll_usb.h"

namespace foc {
    namespace {
        constexpr float kCurKpMax = 20.0f;

        constexpr float kCurKiMax = 200.0f;
    }

    CurrentLoop::CurrentLoop(float kp, float ki, float out_limit)
        : pi_d_(kp, ki, -out_limit, out_limit), pi_q_(kp, ki, -out_limit, out_limit)
    {
    }

    void CurrentLoop::run(float id_ref, float iq_ref, float id_fb, float iq_fb, float dt)
    {
        if (!enabled_) {
            vd_ = 0.0f;
            vq_ = 0.0f;
            return;
        }

        vd_ = pi_d_.run(id_ref, id_fb, dt);
        vq_ = pi_q_.run(iq_ref, iq_fb, dt);
    }

    void CurrentLoop::setGains(float d_kp, float d_ki, float q_kp, float q_ki)
    {
        if (d_kp >= 0.0f) pi_d_.setGains((d_kp > kCurKpMax) ? kCurKpMax : d_kp, -1.0f);
        if (d_ki >= 0.0f) pi_d_.setGains(-1.0f, (d_ki > kCurKiMax) ? kCurKiMax : d_ki);
        if (q_kp >= 0.0f) pi_q_.setGains((q_kp > kCurKpMax) ? kCurKpMax : q_kp, -1.0f);
        if (q_ki >= 0.0f) pi_q_.setGains(-1.0f, (q_ki > kCurKiMax) ? kCurKiMax : q_ki); 

    }

    void CurrentLoop::reset()
    {
        pi_d_.reset();
        pi_q_.reset();
        vd_ = 0.0f;
        vq_ = 0.0f;
    }
}


