#include "curvature_steer_control.h"

#include <stdio.h>
#include <iostream>
#include <cmath>

static const float DEFAULT_DISTANCE_PID_KP = 1.5;
static const float DEFAULT_DISTANCE_PID_KI = 0;
static const float DEFAULT_DISTANCE_PID_KD = 0;
static const float DEFAULT_YAW_PID_KP = 0.3;
static const float DEFAULT_YAW_PID_KI = 0;
static const float DEFAULT_YAW_PID_KD = 0.15;

curvature_steer_control::curvature_steer_control()
{
    this->path_yaw_pid = pid_controller(DEFAULT_YAW_PID_KP, DEFAULT_YAW_PID_KI, DEFAULT_YAW_PID_KD);
    this->path_distance_pid = pid_controller(DEFAULT_DISTANCE_PID_KP, DEFAULT_DISTANCE_PID_KI, DEFAULT_DISTANCE_PID_KD);
    this->yaw_kp = DEFAULT_YAW_PID_KP;
    this->yaw_ki = DEFAULT_YAW_PID_KI;
    this->yaw_kd = DEFAULT_YAW_PID_KD;
    this->yaw_pre_e = 0;
}

curvature_steer_control::curvature_steer_control(const float max_steer_angle, const float max_speed, const float wheel_base, const float center_to_gps_distance = 0)
: path_tracker(max_steer_angle, max_speed, wheel_base, center_to_gps_distance)
{
    this->path_yaw_pid = pid_controller(DEFAULT_YAW_PID_KP, DEFAULT_YAW_PID_KI, DEFAULT_YAW_PID_KD);
    this->path_distance_pid = pid_controller(DEFAULT_DISTANCE_PID_KP, DEFAULT_DISTANCE_PID_KI, DEFAULT_DISTANCE_PID_KD);
    this->yaw_kp = DEFAULT_YAW_PID_KP;
    this->yaw_ki = DEFAULT_YAW_PID_KI;
    this->yaw_kd = DEFAULT_YAW_PID_KD;
}

curvature_steer_control::~curvature_steer_control()
{

}

void curvature_steer_control::set_gain(int gain_index, float* gain_value)
{
    this->yaw_kp = gain_value[0];
    this->yaw_ki = gain_value[1];
    this->yaw_kd = gain_value[2];
}

void curvature_steer_control::get_gain(int gain_index, float* gain_value)
{
    gain_value[0] = this->yaw_kp;
    gain_value[1] = this->yaw_ki;
    gain_value[2] = this->yaw_kd;
}

float curvature_steer_control::steering_control(pt_control_state_t state, std::vector<path_point_t> target_point)
{
    path_point_t current_state_to_point = {state.x, state.y, state.yaw, 0, 0};
    std::vector<path_point_t> circle_paths;
    // float new_yaw = path_tracker::pi_to_pi(state.yaw + state.steer);
    float new_yaw = path_tracker::pi_to_pi(state.yaw);
    float w = 0.1;
    float output;

    for (int i = 0; i < target_point.size(); i++) {
        path_point_t circle_path = path_tracker::get_path_circle(current_state_to_point, target_point[i], tan(path_tracker::pi_to_pi(new_yaw + PT_M_PI_2)));
        circle_paths.push_back(circle_path);
        output += circle_path.k;// * (i * w + 1);
    }

    output /= target_point.size();

    float circle_x_for_direction = circle_paths[0].x - state.x;
    float circle_y_for_direction = circle_paths[0].y - state.y;

    float rotation_y = std::sin(-state.yaw) * circle_x_for_direction + std::cos(-state.yaw) * circle_y_for_direction;
    if (rotation_y < 0) {
        output *= -1;
    }

    float error = output - this->state.steer;
    output = this->state.steer + error * this->yaw_kp + this->yaw_pre_e * this->yaw_kd;

    this->yaw_pre_e = error;

    return output;
}

float curvature_steer_control::velocity_control(pt_control_state_t state, path_point_t target_point)
{
    return target_point.speed;
}
