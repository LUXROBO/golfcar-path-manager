#ifndef LQR_PID_CONTROL_H
#define LQR_PID_CONTROL_H


typedef struct pid_parameter
{
    double p_gain;
    double i_gain;
    double d_gain;
    double target;
    double error;
    double pri_error;
    double error_limit;
    double error_sum;
    double error_sum_limit;
    double output;
} pid_parameter_t;

class pid_controller
{
    public:
        pid_controller();
        pid_controller(double p_gain, double i_gain, double d_gain)
        {
            this->parameter.p_gain = p_gain;
            this->parameter.i_gain = i_gain;
            this->parameter.d_gain = d_gain;
            this->parameter.target = 0;
            this->parameter.error = 0;
            this->parameter.pri_error = 0;
            this->parameter.error_limit = 0;
            this->parameter.error_sum = 0;
            this->parameter.error_sum_limit = 0;
            this->parameter.output = 0;
        }
        pid_controller(pid_parameter_t parameter)
        {
            this->parameter = parameter;
        }
        void init(double p_gain, double i_gain, double d_gain);
        void set_target(double target);
        double calculate(double current_value);

    private:
        pid_parameter_t parameter;
};

#endif
