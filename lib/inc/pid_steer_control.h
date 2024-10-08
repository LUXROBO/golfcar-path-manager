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
    pid_steer_control(const float max_steer_angle, const float max_speed, const float wheel_base, const float center_to_gps_distance);
    ~pid_steer_control();

public:
    virtual void set_gain(int gain_index, float* gain_value);
    virtual void get_gain(int gain_index, float* gain_value);

private:
    virtual float steering_control(pt_control_state_t state, path_point_t target_point[]);
    virtual float velocity_control(pt_control_state_t state, path_point_t target_point);

private:
    pid_controller path_accel_pid;
    pid_controller path_distance_pid;

    float steer_kp;
    float steer_ki;
    float steer_kd;
    float steer_pre_e;

    float adapted_pid_distance_threshold;
    float adapted_pid_distance_gain;
    float adapted_pid_yaw_gain;

public:
};
