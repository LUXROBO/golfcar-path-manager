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
    curvature_steer_control(const float max_steer_angle, const float max_speed, const float wheel_base, const float center_to_gps_distance);
    ~curvature_steer_control();

public:
    virtual void set_gain(int gain_index, float* gain_value);
    virtual void get_gain(int gain_index, float* gain_value);

    path_point_t test_function(path_point_t current, path_point_t target_point);
private:
    virtual float steering_control(pt_control_state_t state, path_point_t target_point);
    virtual float velocity_control(pt_control_state_t state, path_point_t target_point);
private:
    pid_controller path_yaw_pid;
    pid_controller path_distance_pid;

    float yaw_kp;
    float yaw_ki;
    float yaw_kd;
    float yaw_pre_e;

public:
    path_point_t past_path_circle;
};
