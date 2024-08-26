#include "curvature_steer_control.h"

#include <stdio.h>
#include <iostream>
#include <cmath>

static const float DEFAULT_DISTANCE_PID_KP = 1.5;
static const float DEFAULT_DISTANCE_PID_KI = 0;
static const float DEFAULT_DISTANCE_PID_KD = 0;
static const float DEFAULT_YAW_PID_KP = 2.2;
static const float DEFAULT_YAW_PID_KI = 0;
static const float DEFAULT_YAW_PID_KD = 0;

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

}

void curvature_steer_control::get_gain(int gain_index, float* gain_value)
{
    *gain_value = 1;
}

float curvature_steer_control::steering_control(pt_control_state_t state, path_point_t target_point)
{
    path_point_t current_state_to_point = {state.x, state.y, state.yaw, 0, 0};
    // float new_yaw = path_tracker::pi_to_pi(state.yaw + state.steer);
    float new_yaw = path_tracker::pi_to_pi(state.yaw);
    path_point_t circle1 = path_tracker::get_path_circle(current_state_to_point, target_point, tan(path_tracker::pi_to_pi(new_yaw + PT_M_PI_2)));
    path_point_t circle2 = path_tracker::get_path_circle(target_point, current_state_to_point, tan(path_tracker::pi_to_pi(target_point.yaw + PT_M_PI_2)));

    float r1 = 1 / circle1.k;
    float r2 = 1 / circle2.k;

    path_point_t small_circle = circle1;
    float small_circle_r = r1;
    path_point_t big_circle = circle2;
    float big_circle_r = r2;

    if (r1 > r2) {
        small_circle = circle2;
        small_circle_r = r2;
        big_circle = circle1;
        big_circle_r = r1;
    }

    float circle_to_circle = sqrt(pow(small_circle.x - big_circle.x, 2) + pow(small_circle.y - big_circle.y, 2));

    float rr2 = 0;
    float new_point_to_r1_angle = 0;

    // 두 원이 따로 있는경우
    if (circle_to_circle > big_circle_r) {
        rr2 = small_circle_r - circle_to_circle + big_circle_r;
        new_point_to_r1_angle = path_tracker::pi_to_pi(atan2((big_circle.y - small_circle.y), (big_circle.x - small_circle.x)));
    } else { // 큰 원 안에 작은 원이 있는 경우
        rr2 = small_circle_r + circle_to_circle - big_circle_r;
        new_point_to_r1_angle = path_tracker::pi_to_pi(atan2((small_circle.y - big_circle.y), (small_circle.x - big_circle.x)));
    }

    float new_point_to_r1 = small_circle_r - rr2 / 2;
    path_point_t new_circle_point = {small_circle.x + new_point_to_r1 * cos(new_point_to_r1_angle),
                                     small_circle.y + new_point_to_r1 * sin(new_point_to_r1_angle), 0, 0, 0};

    path_point_t circle3 = path_tracker::get_path_circle(new_circle_point, target_point, tan(new_point_to_r1_angle));
    float target_yaw = 0;
    if (circle3.k < 0.0001) {
        this->yaw_error = 0;
    } else {
        float current_to_circle_yaw = atan2(state.y - circle3.y, state.x - circle3.x);
        float current_to_circle_center_distance = 1 / circle3.k;
        float target_yaw1 = path_tracker::pi_to_pi(current_to_circle_yaw + PT_M_PI_2);
        float target_yaw2 = path_tracker::pi_to_pi(current_to_circle_yaw - PT_M_PI_2);

        if (fabsf(path_tracker::pi_to_pi(target_yaw1 - (new_yaw))) > fabsf(path_tracker::pi_to_pi(target_yaw2 - (new_yaw)))) {
            target_yaw = target_yaw2;
        } else {
            target_yaw = target_yaw1;
        }
        this->yaw_error = this->pi_to_pi(target_yaw - (new_yaw));
    }

    float past_circle_to_current = sqrt(pow((this->past_path_circle.x - state.x), 2) + pow((this->past_path_circle.y - state.y), 2));

    this->path_distance_pid.set_target(0);
    float steer_delta = atan2(this->path_distance_pid.calculate(past_circle_to_current), state.v);

    this->past_path_circle = circle3;

    float steer_delta2 = this->yaw_error * this->yaw_kp + (this->yaw_error - this->yaw_pre_e) * this->yaw_kd;
    this->yaw_pre_e = this->yaw_error;

    if (fabsf(state.v) > 0.01) {
        float temp_steer = atan(steer_delta2 * this->wheel_base / state.v);
        int count = 0;
        float offset = 2 * PT_M_PI / 180;
        while (count <= 10) {
            pt_control_state_t current_state = state;
            current_state.steer = temp_steer;

            for (int i = 0; i < 10; i++) {
                float v_to_g_slope_diff_angle = std::atan(this->lr * std::tan(current_state.steer) / this->wheel_base);
                current_state.yaw += current_state.v * 0.1 * std::cos(v_to_g_slope_diff_angle) * std::tan(v_to_g_slope_diff_angle) / this->wheel_base;
                current_state.yaw = path_tracker::pi_to_pi(current_state.yaw);
            }
            float temp_yaw_error = path_tracker::pi_to_pi(target_yaw - current_state.yaw);
            if (fabsf(temp_yaw_error) < 0.01) {
                break;
            } else {
                if (temp_yaw_error > 0) {
                    temp_steer += offset * (10 - count) / 10;
                } else {
                    temp_steer -= offset * (10 - count) / 10;
                }
            }
            count += 1;
        }
        steer_delta2 = temp_steer;
    }

    return steer_delta2;
}

float curvature_steer_control::velocity_control(pt_control_state_t state, path_point_t target_point)
{
    return target_point.speed;
}
