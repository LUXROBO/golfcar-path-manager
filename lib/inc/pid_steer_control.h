#pragma once

// std
#include <vector>
#include <fstream>


#include "lqr_pid_control.h"
#include "path_manager.h"


class pid_steer_control : public path_tracker
{
public:
    pid_steer_control();
    pid_steer_control(const double max_steer_angle, const double max_speed, const double wheel_base, const double center_to_gps_distance);
    ~pid_steer_control();

public:
    virtual void set_gain(int gain_index, double* gain_value);
    virtual void get_gain(int gain_index, double* gain_value);

private:
    virtual double steering_control(pt_control_state_t state, path_point_t target_point);
    virtual double velocity_control(pt_control_state_t state, path_point_t target_point);

private:
    pid_controller path_accel_pid;
    pid_controller path_distance_pid;

    double steer_kp;
    double steer_ki;
    double steer_kd;
    double steer_pre_e;

    double adapted_pid_distance_threshold;
    double adapted_pid_distance_gain;
    double adapted_pid_yaw_gain;

public:
};
