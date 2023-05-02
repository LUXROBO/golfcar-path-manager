#ifndef LQR_PID_CONTROL_H
#define LQR_PID_CONTROL_H

typedef struct pid_parameter_ {
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
            this->parameter.p_gain = p_gain;
            this->parameter.i_gain = i_gain;
            this->parameter.d_gain = d_gain;
        }
        pid_controller(pid_parameter_t parameter)
        {
            this->parameter = parameter;
        }
        void init(float p_gain, float i_gain, float d_gain);
        void set_target(float target);
        float calculate(float current_value);

    private:
        pid_parameter_t parameter;
};

#endif
