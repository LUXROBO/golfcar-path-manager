#ifndef LQR_STEER_CONTROL_H
#define LQR_STEER_CONTROL_H

#include <vector>
#include <fstream>

#include "lqr_pid_control.h"
#include "path_manager.h"


class lqr_steer_control : public path_tracking_controller
{
public:
    enum lqr_gain_select {
        q = 0,
        r = 1
    };
    lqr_steer_control();
    ~lqr_steer_control();

private:
    ModelMatrix dlqr(ModelMatrix A, ModelMatrix B, ModelMatrix Q, ModelMatrix R);
    int lqr_steering_control(ControlState state, double& steer, double& pe, double& pth_e);
    ModelMatrix solve_DARE(ModelMatrix A, ModelMatrix B, ModelMatrix Q, ModelMatrix R);
    virtual int steering_control(ControlState state, double& steer);
    virtual int velocity_control(ControlState state, double& accel);

private:
    ModelMatrix Q;
    ModelMatrix R;

public :
    virtual void set_gain(int gain_index, double* gain_value);
    virtual void get_gain(int gain_index, double* gain_value);
};

#endif // LQR_STEER_CONTROL_H
