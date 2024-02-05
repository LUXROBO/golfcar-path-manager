#include "curvature_steer_control.h"

#include <stdio.h>
#include <iostream>
#include <cmath>

static const double DEFAULT_DISTANCE_PID_KP = 1;
static const double DEFAULT_DISTANCE_PID_KI = 0;
static const double DEFAULT_DISTANCE_PID_KD = 0;
static const double DEFAULT_YAW_PID_KP = 1.5;
static const double DEFAULT_YAW_PID_KI = 0;
static const double DEFAULT_YAW_PID_KD = 0;

curvature_steer_control::curvature_steer_control()
{
    this->path_yaw_pid = pid_controller(DEFAULT_YAW_PID_KP, DEFAULT_YAW_PID_KI, DEFAULT_YAW_PID_KD);
    this->path_distance_pid = pid_controller(DEFAULT_DISTANCE_PID_KP, DEFAULT_DISTANCE_PID_KI, DEFAULT_DISTANCE_PID_KD);
    this->yaw_kp = DEFAULT_YAW_PID_KP;
    this->yaw_ki = DEFAULT_YAW_PID_KI;
    this->yaw_kd = DEFAULT_YAW_PID_KD;
    this->yaw_pre_e = 0;
    this->lf = 1.075 - 0.77;
    this->lr = 1.075 + 0.77;
}

curvature_steer_control::curvature_steer_control(const double max_steer_angle, const double max_speed, const double wheel_base)
: path_tracker(max_steer_angle, max_speed, wheel_base)
{
    this->path_yaw_pid = pid_controller(DEFAULT_YAW_PID_KP, DEFAULT_YAW_PID_KI, DEFAULT_YAW_PID_KD);
    this->path_distance_pid = pid_controller(DEFAULT_DISTANCE_PID_KP, DEFAULT_DISTANCE_PID_KI, DEFAULT_DISTANCE_PID_KD);
    this->yaw_kp = DEFAULT_YAW_PID_KP;
    this->yaw_ki = DEFAULT_YAW_PID_KI;
    this->yaw_kd = DEFAULT_YAW_PID_KD;
    this->lf = 1.075 - 0.77;
    this->lr = 1.075 + 0.77;
}

curvature_steer_control::~curvature_steer_control()
{

}

void curvature_steer_control::set_gain(int gain_index, double* gain_value)
{

}

void curvature_steer_control::get_gain(int gain_index, double* gain_value)
{

}

double curvature_steer_control::steering_control(pt_control_state_t state, path_point_t target_point)
{
    path_point_t current_state_to_point = {state.x, state.y, state.yaw, 0, 0};
    // double new_yaw = path_tracker::pi_to_pi(state.yaw + state.steer);
    double new_yaw = path_tracker::pi_to_pi(state.yaw);
    path_point_t circle1 = path_tracker::get_path_circle(current_state_to_point, target_point, tan(path_tracker::pi_to_pi(new_yaw + PT_M_PI_2)));
    path_point_t circle2 = path_tracker::get_path_circle(target_point, current_state_to_point, tan(path_tracker::pi_to_pi(target_point.yaw + PT_M_PI_2)));

    double r1 = 1 / circle1.k;
    double r2 = 1 / circle2.k;

    path_point_t small_circle = circle1;
    double small_circle_r = r1;
    path_point_t big_circle = circle2;
    double big_circle_r = r2;

    if (r1 > r2) {
        small_circle = circle2;
        small_circle_r = r2;
        big_circle = circle1;
        big_circle_r = r1;
    }

    double circle_to_circle = sqrt(pow(small_circle.x - big_circle.x, 2) + pow(small_circle.y - big_circle.y, 2));

    double rr2 = 0;
    double new_point_to_r1_angle = 0;

    // 두 원이 따로 있는경우
    if (circle_to_circle > big_circle_r) {
        rr2 = small_circle_r - circle_to_circle + big_circle_r;
        new_point_to_r1_angle = path_tracker::pi_to_pi(atan2((big_circle.y - small_circle.y), (big_circle.x - small_circle.x)));
    } else { // 큰 원 안에 작은 원이 있는 경우
        rr2 = small_circle_r + circle_to_circle - big_circle_r;
        new_point_to_r1_angle = path_tracker::pi_to_pi(atan2((small_circle.y - big_circle.y), (small_circle.x - big_circle.x)));
    }

    double new_point_to_r1 = small_circle_r - rr2 / 2;
    path_point_t new_circle_point = {small_circle.x + new_point_to_r1 * cos(new_point_to_r1_angle),
                                     small_circle.y + new_point_to_r1 * sin(new_point_to_r1_angle), 0, 0, 0};

    path_point_t circle3 = path_tracker::get_path_circle(new_circle_point, target_point, tan(new_point_to_r1_angle));
    double target_yaw = 0;
    if (circle3.k < 0.0001) {
        this->yaw_error = 0;
    } else {
        double current_to_circle_yaw = atan2(state.y - circle3.y, state.x - circle3.x);
        double current_to_circle_center_distance = 1 / circle3.k;
        double target_yaw1 = path_tracker::pi_to_pi(current_to_circle_yaw + PT_M_PI_2);
        double target_yaw2 = path_tracker::pi_to_pi(current_to_circle_yaw - PT_M_PI_2);

        if (fabsf(path_tracker::pi_to_pi(target_yaw1 - (new_yaw))) > fabsf(path_tracker::pi_to_pi(target_yaw2 - (new_yaw)))) {
            target_yaw = target_yaw2;
        } else {
            target_yaw = target_yaw1;
        }
        this->yaw_error = this->pi_to_pi(target_yaw - (new_yaw));
    }

    double past_circle_to_current = sqrt(pow((this->past_path_circle.x - state.x), 2) + pow((this->past_path_circle.y - state.y), 2));

    this->path_distance_pid.set_target(0);
    double steer_delta = atan2(this->path_distance_pid.calculate(past_circle_to_current), state.v);

    this->past_path_circle = circle3;

    double steer_delta2 = this->yaw_error * this->yaw_kp + (this->yaw_error - this->yaw_pre_e) * this->yaw_kd;
    this->yaw_pre_e = this->yaw_error;

    double v_to_g_slope_diff_angle = std::asin(this->yaw_error * this->lr / state.v);
    steer_delta2 = std::atan(std::tan(v_to_g_slope_diff_angle) * this->lr / this->wheel_base);

    return steer_delta2;
}

double curvature_steer_control::velocity_control(pt_control_state_t state, path_point_t target_point)
{
    return target_point.speed;
}

path_point_t curvature_steer_control::test_function(path_point_t state, path_point_t target_point)
{
    path_point_t current_state_to_point = {state.x, state.y, state.yaw, 0, 0};
    double new_yaw = path_tracker::pi_to_pi(state.yaw);
    path_point_t circle1 = path_tracker::get_path_circle(current_state_to_point, target_point, tan(path_tracker::pi_to_pi(new_yaw + PT_M_PI_2)));
    path_point_t circle2 = path_tracker::get_path_circle(target_point, current_state_to_point, tan(path_tracker::pi_to_pi(target_point.yaw + PT_M_PI_2)));

    double r1 = 1 / circle1.k;
    double r2 = 1 / circle2.k;

    path_point_t small_circle = circle1;
    double small_circle_r = r1;
    path_point_t big_circle = circle2;
    double big_circle_r = r2;

    if (r1 > r2) {
        small_circle = circle2;
        small_circle_r = r2;
        big_circle = circle1;
        big_circle_r = r1;
    }

    double circle_to_circle = sqrt(pow(small_circle.x - big_circle.x, 2) + pow(small_circle.y - big_circle.y, 2));

    double rr2 = 0;
    double new_point_to_r1_angle = 0;

    // 두 원이 따로 있는경우
    if (circle_to_circle > big_circle_r) {
        rr2 = small_circle_r - circle_to_circle + big_circle_r;
        new_point_to_r1_angle = path_tracker::pi_to_pi(atan2((big_circle.y - small_circle.y), (big_circle.x - small_circle.x)));
    } else { // 큰 원 안에 작은 원이 있는 경우
        rr2 = small_circle_r + circle_to_circle - big_circle_r;
        new_point_to_r1_angle = path_tracker::pi_to_pi(atan2((small_circle.y - big_circle.y), (small_circle.x - big_circle.x)));
    }

    double new_point_to_r1 = small_circle_r - rr2 / 2;
    path_point_t new_circle_point = {small_circle.x + new_point_to_r1 * cos(new_point_to_r1_angle),
                                     small_circle.y + new_point_to_r1 * sin(new_point_to_r1_angle), 0, 0, 0};

    path_point_t circle3 = path_tracker::get_path_circle(new_circle_point, target_point, tan(new_point_to_r1_angle));

    double current_to_circle_yaw = atan2(state.y - circle3.y, state.x - circle3.x);
    double current_to_circle_center_distance = 1 / circle3.k;
    double target_yaw1 = path_tracker::pi_to_pi(current_to_circle_yaw + PT_M_PI_2);
    double target_yaw2 = path_tracker::pi_to_pi(current_to_circle_yaw - PT_M_PI_2);
    double target_yaw = 0;

    if (fabsf(path_tracker::pi_to_pi(target_yaw1 - (new_yaw))) > fabsf(path_tracker::pi_to_pi(target_yaw2 - (new_yaw)))) {
        target_yaw = target_yaw2;
    } else {
        target_yaw = target_yaw1;
    }
    this->yaw_error = this->pi_to_pi(target_yaw - (new_yaw));

    this->past_path_circle = circle3;

    double steer_delta2 = path_tracker::pi_to_pi(this->yaw_error * this->yaw_kp + (this->yaw_error - this->yaw_pre_e) * this->yaw_kd);
    this->yaw_pre_e = this->yaw_error;

    double steer = -atan(steer_delta2 * this->wheel_base / 1.6);

    return circle3;
}
