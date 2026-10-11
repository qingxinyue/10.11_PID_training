//
// Created by qingx on 2026/10/11.
//

#ifndef INC_1011_PID_PID_HPP
#define INC_1011_PID_PID_HPP


class PID
{
public:
    PID(float kp, float ki, float kd,float i_max, float out_max, float d_filter_k =1);
    PID(void):PID(0,0,0,0,0,1){}
    void reset();
    float calc(float ref, float fdb);
    float limit(float cur_err_sum, float min_limit, float max_limit);

    float ki_,kd_,kp_;
    float i_max_,out_max_,d_filter_k_;
private:
    float ref_;
    float fdb_;
    float err_;
    float err_sum_;
    float last_err_;
    float output_;
    float last_dout_;
    float pout_;
    float iout_;
    float dout_;
};


#endif //INC_1011_PID_PID_HPP
