//
// Created by qingx on 2026/10/11.
//

#include "PID.hpp"
PID::PID(float kp, float ki, float kd,float i_max, float out_max, float d_filter_k)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    i_max_ = i_max;
    out_max_ = out_max;
    d_filter_k_ = d_filter_k;
    reset();
}
void PID::reset()
{
    ref_ =0.0f;
    fdb_ =0.0f;
    err_ = ref_ -fdb_;
    last_err_ =0.0f;
    last_dout_ =0.0f;
    err_sum_ = 0.0f;
    pout_ = 0.0f;
    iout_ = 0.0f;
    dout_ = 0.0f;
    output_ = 0.0f;
}

float PID::limit(float cur_err_sum, float min_limit, float max_limit)
{
    if (cur_err_sum < min_limit) return min_limit;
    if (cur_err_sum >max_limit) return max_limit;
    return cur_err_sum;
}

float PID::calc(float ref, float fdb)
{
    ref_ = ref;
    fdb_ = fdb;
    err_ = ref_ - fdb_;
    pout_ = err_ *kp_;
    err_sum_ =limit(err_sum_+err_, -i_max_, i_max_);
    iout_ =err_sum_ *ki_;
    dout_ =(err_ -last_err_)*kd_*d_filter_k_ + last_dout_*(1- d_filter_k_);

    output_ =pout_ +iout_ +dout_;
    output_ = limit(pout_ + iout_ + dout_, -out_max_, out_max_);

    last_err_ = err_;
    last_dout_ = dout_;

    return output_;
}