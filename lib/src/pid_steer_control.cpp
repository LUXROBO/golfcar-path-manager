#include "pid_steer_control.h"

#include <stdio.h>
#include <iostream>
#include <cmath>


static const float DEFAULT_DISTANCE_PID_KP = 0.6;
static const float DEFAULT_DISTANCE_PID_KI = 0.0;
static const float DEFAULT_DISTANCE_PID_KD = 0.8;
static const float DEFAULT_STEER_PID_KP = 1.0;
static const float DEFAULT_STEER_PID_KI = 0.0;
static const float DEFAULT_STEER_PID_KD = 0.9;

static const float DEFAULT_ADAPTED_PID_DISTANCE_THRESHOLD = 0.25;
static const float DEFAULT_ADAPTED_PID_DISTANCE_GAIN = 1.3;
static const float DEFAULT_ADAPTED_PID_YAW_GAIN = 1;

pid_steer_control::pid_steer_control()
{
    this->path_accel_pid = pid_controller(1, 0, 0);
    this->path_distance_pid = pid_controller(DEFAULT_DISTANCE_PID_KP, DEFAULT_DISTANCE_PID_KI, DEFAULT_DISTANCE_PID_KD);

    this->steer_kp = DEFAULT_STEER_PID_KP;
    this->steer_ki = DEFAULT_STEER_PID_KI;
    this->steer_kd = DEFAULT_STEER_PID_KD;
    this->steer_pre_e = 0;

    this->adapted_pid_distance_threshold = DEFAULT_ADAPTED_PID_DISTANCE_THRESHOLD;
    this->adapted_pid_distance_gain = DEFAULT_ADAPTED_PID_DISTANCE_GAIN;
    this->adapted_pid_yaw_gain = DEFAULT_ADAPTED_PID_YAW_GAIN;
}

pid_steer_control::pid_steer_control(const float max_steer_angle, const float max_speed, const float wheel_base, const float center_to_gps_distance = 0)
: path_tracker(max_steer_angle, max_speed, wheel_base, center_to_gps_distance)
{
    this->path_accel_pid = pid_controller(1, 0, 0);
    this->path_distance_pid = pid_controller(DEFAULT_DISTANCE_PID_KP, DEFAULT_DISTANCE_PID_KI, DEFAULT_DISTANCE_PID_KD);

    this->steer_kp = DEFAULT_STEER_PID_KP;
    this->steer_ki = DEFAULT_STEER_PID_KI;
    this->steer_kd = DEFAULT_STEER_PID_KD;
    this->steer_pre_e = 0;

    this->adapted_pid_distance_threshold = DEFAULT_ADAPTED_PID_DISTANCE_THRESHOLD;
    this->adapted_pid_distance_gain = DEFAULT_ADAPTED_PID_DISTANCE_GAIN;
    this->adapted_pid_yaw_gain = DEFAULT_ADAPTED_PID_YAW_GAIN;
}

pid_steer_control::~pid_steer_control()
{

}

void pid_steer_control::set_gain(int gain_index, float* gain_value)
{
    if (gain_index == PT_GAIN_TYPE_PID_DISTANCE) {
        this->path_distance_pid.set_gain(gain_value[0], gain_value[1], gain_value[2]);
    } else if (gain_index == PT_GAIN_TYPE_PID_STEER) {
        this->steer_kp = gain_value[0];
        this->steer_ki = gain_value[1];
        this->steer_kd = gain_value[2];
    } else {

    }
}

void pid_steer_control::get_gain(int gain_index, float* gain_value)
{
    if (gain_index == PT_GAIN_TYPE_PID_DISTANCE) {
        gain_value[0] = this->path_distance_pid.get_p_gain();
        gain_value[1] = this->path_distance_pid.get_i_gain();
        gain_value[2] = this->path_distance_pid.get_d_gain();
    } else if (gain_index == PT_GAIN_TYPE_PID_STEER) {
        gain_value[0] = this->steer_kp;
        gain_value[1] = this->steer_ki;
        gain_value[2] = this->steer_kd;
    } else {

    }
}

float pid_steer_control::steering_control(pt_control_state_t state, path_point_t target_point)
{
    float th_e = pi_to_pi(this->yaw_error * this->steer_kp + (this->yaw_error - this->steer_pre_e) * this->steer_kd);

    this->path_distance_pid.set_target(this->distance_error);
    float steer_delta = std::atan2(this->path_distance_pid.calculate(0), 1.6);

    float steer = th_e + steer_delta;

    steer_pre_e = this->yaw_error;

    return steer;
}

float pid_steer_control::velocity_control(pt_control_state_t state, path_point_t target_point)
{

    return target_point.speed;
}
