#include "pid_steer_control.h"

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <algorithm>


static const double DEFAULT_MAX_STEER = 45.0 * M_PI / 180.0;     // [rad] 45deg
static const double DEFAULT_MAX_SPEED = 10.0 / 3.6;              // [ms] 10km/h
static const double DEFAULT_WHEEL_BASE = 0.41;                   // 앞 뒤 바퀴 사이 거리 [m]
static const double DEFAULT_DISTANCE_PID_KP = 0.4;                        // gain
static const double DEFAULT_DISTANCE_PID_KI = 0.05;
static const double DEFAULT_DISTANCE_PID_KD = 0.7;
static const double DEFAULT_STEER_PID_KP = 1.2;
static const double DEFAULT_STEER_PID_KI = 0.0;
static const double DEFAULT_STEER_PID_KD = 0.9;

static const double DEFAULT_ADAPTED_PID_DISTANCE_THRESHOLD = 0.25;
static const double DEFAULT_ADAPTED_PID_DISTANCE_GAIN = 1.3;
static const double DEFAULT_ADAPTED_PID_YAW_GAIN = 1;

int pid_steer_control::test_funtion()
{
    // std::cout << "test" << std::endl;
    return 0;
}

pid_steer_control::pid_steer_control()
{
    this->init(DEFAULT_MAX_STEER, DEFAULT_MAX_SPEED, DEFAULT_WHEEL_BASE);
    this->path_accel_pid = pid_controller(1, 0, 0);
    this->path_distance_pid = pid_controller(DEFAULT_DISTANCE_PID_KP, DEFAULT_DISTANCE_PID_KI, DEFAULT_DISTANCE_PID_KD);
    this->points.clear();
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

int pid_steer_control::steering_control(ControlState state, double& steer)
{
    int current_target_ind = this->calculate_target_index(state, this->points, this->target_ind);
    int jumped_point = current_target_ind + this->jumping_point;
    double jumped_distance_error = 0;
    if (current_target_ind != -1) {
        if (current_target_ind < this->target_ind) {
            current_target_ind = this->target_ind;
        }
        if (jumped_point >= this->points.size()) {
            jumped_point = current_target_ind;
        }

        if (jumped_point != 0) {
            Point current_state = {this->state.x, this->state.y, 0, 0, 0};
            jumped_distance_error = distance_between_point_and_line(current_state,
                                                                    this->points[jumped_point - 1],
                                                                    this->points[jumped_point]);
        } else {
            jumped_distance_error = 0;
        }

        this->yaw_error = pi_2_pi(this->points[jumped_point].yaw - state.yaw);
        if (this->adapted_pid_distance_threshold > abs(this->distance_error)) {
            this->distance_error *= this->adapted_pid_distance_gain;
        } else {
            this->yaw_error = pi_2_pi(this->yaw_error * this->adapted_pid_yaw_gain);
        }

        double th_e = pi_2_pi(this->yaw_error * this->steer_kp + (this->yaw_error - this->steer_pre_e) * this->steer_kd);

        this->path_distance_pid.set_target(this->distance_error);
        double steer_delta = std::atan2(this->path_distance_pid.calculate(0), 1.6);

        steer = th_e + steer_delta;

        steer_pre_e = this->yaw_error;
    }

    return current_target_ind;
}

int pid_steer_control::velocity_control(ControlState state, double& accel)
{
    this->path_accel_pid.set_target(this->points[this->target_ind].speed);
    accel = this->path_accel_pid.calculate(this->state.v);

    return 0;
}

void pid_steer_control::set_gain(int gain_index, double* gain_value)
{
    if (gain_index == PATH_TRACKER_PID_TYPE_DISTANCE) {
        this->path_distance_pid.set_gain(gain_value[0], gain_value[1], gain_value[2]);
    } else if (gain_index == PATH_TRACKER_PID_TYPE_STEER) {
        this->steer_kp = gain_value[0];
        this->steer_ki = gain_value[1];
        this->steer_kd = gain_value[2];
    } else {

    }
}

void pid_steer_control::get_gain(int gain_index, double* gain_value)
{
    if (gain_index == PATH_TRACKER_PID_TYPE_DISTANCE) {
        gain_value[0] = this->path_distance_pid.get_p_gain();
        gain_value[1] = this->path_distance_pid.get_i_gain();
        gain_value[2] = this->path_distance_pid.get_d_gain();
    } else if (gain_index == PATH_TRACKER_PID_TYPE_STEER) {
        gain_value[0] = this->steer_kp;
        gain_value[1] = this->steer_ki;
        gain_value[2] = this->steer_kd;
    } else {

    }
}

