#include "lqr_pid_control.h"


pid_controller::pid_controller()
{
    this->parameter.p_gain = 0;
    this->parameter.i_gain = 0;
    this->parameter.d_gain = 0;
    this->parameter.error = this->parameter.pri_error = this->parameter.error_sum = 0;
    this->parameter.target = 0;
    this->parameter.output = 0;
    this->parameter.error_sum_limit = 1.0;
}

void pid_controller::init(float p_gain, float i_gain, float d_gain)
{
    this->parameter.p_gain = p_gain;
    this->parameter.i_gain = i_gain;
    this->parameter.d_gain = d_gain;
    this->parameter.error = this->parameter.pri_error = this->parameter.error_sum = 0;
    this->parameter.target = 0;
    this->parameter.output = 0;
    this->parameter.error_sum_limit = 1.0;
}

void pid_controller::set_gain(float p_gain, float i_gain, float d_gain)
{
    this->parameter.p_gain = p_gain;
    this->parameter.i_gain = i_gain;
    this->parameter.d_gain = d_gain;
    this->parameter.error_sum = 0;
}

void pid_controller::set_target(float target)
{
    this->parameter.target = target;
}

float pid_controller::calculate(float current_value)
{
    this->parameter.error = this->parameter.target - current_value;

    this->parameter.error_sum += this->parameter.error;
    if (this->parameter.error_sum > this->parameter.error_sum_limit) {
        this->parameter.error_sum = this->parameter.error_sum_limit;
    } else if (this->parameter.error_sum < -this->parameter.error_sum_limit) {
        this->parameter.error_sum = -this->parameter.error_sum_limit;
    }

    this->parameter.output = (this->parameter.p_gain * this->parameter.error) +
                             (this->parameter.i_gain * this->parameter.error_sum) +
                             (this->parameter.d_gain * (this->parameter.error - this->parameter.pri_error));

    this->parameter.pri_error = this->parameter.error;

    return this->parameter.output;
}
