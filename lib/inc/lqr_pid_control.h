#pragma once

typedef struct pid_parameter
{
    float p_gain;
    float i_gain;
    float d_gain;
    float target;
    float error;
    float pri_error;
    float error_limit;
    float error_sum;
    float error_sum_limit;
    float output;
} pid_parameter_t;

class pid_controller
{
    public:
        pid_controller();
        pid_controller(float p_gain, float i_gain, float d_gain)
        {
            init(p_gain, i_gain, d_gain);
        }
        pid_controller(pid_parameter_t parameter)
        {
            this->parameter = parameter;
        }
        void init(float p_gain, float i_gain, float d_gain);
        void set_gain(float p_gain, float i_gain, float d_gain);
        void set_target(float target);
        float get_p_gain() {
            return this->parameter.p_gain;
        };
        float get_i_gain() {
            return this->parameter.i_gain;
        };
        float get_d_gain() {
            return this->parameter.d_gain;
        };
        float calculate(float current_value);

    private:
        pid_parameter_t parameter;
};
