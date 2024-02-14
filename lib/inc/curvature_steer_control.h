#pragma once

// std
#include <vector>
#include <fstream>


#include "lqr_pid_control.h"
#include "path_manager.h"


class curvature_steer_control : public path_tracker
{
public:
    curvature_steer_control();
    curvature_steer_control(const double max_steer_angle, const double max_speed, const double wheel_base, const double center_to_gps_distance);
    ~curvature_steer_control();

public:
    virtual void set_gain(int gain_index, double* gain_value);
    virtual void get_gain(int gain_index, double* gain_value);

    path_point_t test_function(path_point_t current, path_point_t target_point);
private:
    virtual double steering_control(pt_control_state_t state, path_point_t target_point);
    virtual double velocity_control(pt_control_state_t state, path_point_t target_point);
private:
    pid_controller path_yaw_pid;
    pid_controller path_distance_pid;

    double yaw_kp;
    double yaw_ki;
    double yaw_kd;
    double yaw_pre_e;

public:
    path_point_t past_path_circle;
};
