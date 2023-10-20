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

static double distance_between_point_and_line(Point point, Point line_point1, Point line_point2)
{
    double a = (line_point1.y - line_point2.y) / (line_point1.x - line_point2.x);
    double c = line_point1.y - a * line_point1.x;
    double b = -1;

    double error_distance = abs(a * point.x + b * point.y + c) / sqrt(a * a + b * b);

    if (point.y > (a * point.x + c)) { // 그래프 위쪽
        if (line_point2.x > line_point1.x) { // 진행 방향 오른쪽
            error_distance *= -1;
        }
    } else { // 그래프 아래쪽
        if (line_point2.x < line_point1.x) { // 진행 방향 오른쪽
            error_distance *= -1;
        }
    }

    return error_distance;
}

int pid_steer_control::test_funtion()
{
    std::cout << "testtt" << std::endl;
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
}

pid_steer_control::~pid_steer_control()
{

}

int pid_steer_control::steering_control(ControlState state, double& steer)
{
    int current_target_ind = this->calculate_target_index(state, this->points, this->target_ind);
    if (current_target_ind != -1) {
        if (current_target_ind < this->target_ind) {
            current_target_ind = this->target_ind;
        }

        double yaw_error = pi_2_pi(this->points[current_target_ind].yaw - state.yaw);
        double th_e = pi_2_pi(yaw_error * this->steer_kp + (yaw_error - this->steer_pre_e) * this->steer_kd);

        this->path_distance_pid.set_target(this->distance_error);
        double steer_delta = std::atan2(this->path_distance_pid.calculate(0), state.v);

        steer = th_e + steer_delta;

        steer_pre_e = yaw_error;
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
        // this->steer_ki = gain_value[1];
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

