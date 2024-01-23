#pragma once

// std
#include <vector>
#include <fstream>

#include "lqr_pid_control.h"
#include "path_manager.h"
#include "model_matrix.h"


class lqr_steer_control : public path_tracker
{
public:
    lqr_steer_control();
    lqr_steer_control(const double max_steer_angle, const double max_speed, const double wheel_base);
    ~lqr_steer_control();

public:
    virtual double steering_control(pt_control_state_t state, path_point_t target_point);
    virtual double velocity_control(pt_control_state_t state, path_point_t target_point);

    virtual void set_gain(int gain_index, double* gain_value);
    virtual void get_gain(int gain_index, double* gain_value);

private:
    ModelMatrix dlqr(ModelMatrix A, ModelMatrix B, ModelMatrix Q, ModelMatrix R);
    ModelMatrix solve_DARE(ModelMatrix A, ModelMatrix B, ModelMatrix Q, ModelMatrix R);

private:
    ModelMatrix Q;
    ModelMatrix R;
};
