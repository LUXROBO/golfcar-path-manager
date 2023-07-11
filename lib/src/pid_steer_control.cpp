#include "pid_steer_control.h"

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <algorithm>


static const double DEFAULT_MAX_STEER = 45.0 * M_PI / 180.0;     // [rad] 45deg
static const double DEFAULT_MAX_SPEED = 10.0 / 3.6;              // [ms] 10km/h
static const double DEFAULT_WHEEL_BASE = 0.41;                   // 앞 뒤 바퀴 사이 거리 [m]
static const double DEFAULT_DISTANCE_PID_KP = 0.3;                        // gain
static const double DEFAULT_DISTANCE_PID_KI = 0.0;
static const double DEFAULT_DISTANCE_PID_KD = 0.1;
static const double DEFAULT_STEER_PID_KP = 1.0;
static const double DEFAULT_STEER_PID_KI = 0.0;
static const double DEFAULT_STEER_PID_KD = 0.0;

static double distance_between_point_and_line(Point point, Point line_point1, Point line_point2)
{
    double a = (line_point1.y - line_point2.y) / (line_point1.x - line_point2.x);
    double c = line_point1.y - a * line_point1.x;
    double b = -1;

    return abs(a * point.x + b * point.y + c) / sqrt(a * a + b * b);
}

pid_steer_control::pid_steer_control()
{
    this->init(DEFAULT_MAX_STEER, DEFAULT_MAX_SPEED, DEFAULT_WHEEL_BASE);
}

pid_steer_control::~pid_steer_control()
{

}

void pid_steer_control::init(const double max_steer_angle, const double max_speed, const double wheel_base)
{
    this->path_accel_pid = pid_controller(1, 0, 0);
    this->steer_kp = DEFAULT_STEER_PID_KP;
    this->steer_ki = DEFAULT_STEER_PID_KI;
    this->steer_kd = DEFAULT_STEER_PID_KD;
    this->path_distance_pid = pid_controller(DEFAULT_DISTANCE_PID_KP, DEFAULT_DISTANCE_PID_KI, DEFAULT_DISTANCE_PID_KD);
    this->points.clear();

    this->max_steer_angle = max_steer_angle;
    this->max_speed = max_speed;
    this->wheel_base = wheel_base;
}

void pid_steer_control::generate_spline(ControlState init_state, std::vector<WayPoint> waypoints, double target_speed, double ds)
{
    CubicSpline2D spline(waypoints);
    std::vector<Point> points = spline.generate_spline_course(target_speed, ds); // get spline points
    this->set_course(init_state, points);
}

void pid_steer_control::set_course(ControlState init_state, std::vector<Point> points)
{
    int goal_index = points.size() - 1;

    this->init_state = init_state;
    this->points = points;

    if (this->init_state.yaw - this->points[0].yaw >= M_PI) {
        this->init_state.yaw -= 2.0f * M_PI;
    } else if (this->init_state.yaw - this->points[0].yaw <= -M_PI) {
        this->init_state.yaw += 2.0f * M_PI;
    }

    this->goal_state = ControlState(this->points[goal_index].x, this->points[goal_index].y, this->points[goal_index].yaw, 0, this->points[goal_index].speed);
    this->t = 0.0f;

    this->target_ind = this->calculate_nearest_index(this->init_state, this->points, 0);
    this->smooth_yaw(this->points);
    this->oa.clear();
    this->odelta.clear();
}

void pid_steer_control::add_course(ControlState init_state, std::vector<Point> points)
{
    this->points.insert(end(this->points), begin(points), end(points));

    int goal_index = points.size() - 1;

    this->init_state = init_state;  // init start state
    if (this->init_state.yaw - this->points[0].yaw >= M_PI) {
        this->init_state.yaw -= 2.0f * M_PI;
    } else if (this->init_state.yaw - this->points[0].yaw <= -M_PI) {
        this->init_state.yaw += 2.0f * M_PI;
    }
    this->goal_state = ControlState(this->points[goal_index].x, this->points[goal_index].y, this->points[goal_index].yaw, 0, this->points[goal_index].speed);
    this->t = 0.0f;

    this->target_ind = this->calculate_nearest_index(this->init_state, this->points, 0);
    this->smooth_yaw(this->points);
    this->oa.clear();
    this->odelta.clear();
    this->points.back().speed = 2.2;
}

int pid_steer_control::calculate_nearest_index(ControlState state, std::vector<Point> points, int pind)
{
    const int N_IND_SEARCH = 5;
    double min = 10000;
    uint32_t min_point_index = 0;
    for (uint32_t i = pind; i < (pind + N_IND_SEARCH); i++) {
        double dx = state.x - points[i].x;
        double dy = state.y - points[i].y;
        double point_to_distance = dx * dx + dy * dy;
        if (min > point_to_distance) {
            min = point_to_distance;
            min_point_index = i;
        }
        if (i >= (points.size() - 1)) {
            break;
        }
    }
    return min_point_index;
}

int pid_steer_control::calculate_target_index(ControlState state, std::vector<Point> points, int pind, double& err_front_axel)
{
    const int N_IND_SEARCH = 5;
    double min = 10000;
    double target_dx = 0;
    double target_dy = 0;
    uint32_t min_point_index = 0;

    double fx = state.x + this->wheel_base * std::cos(state.yaw);
    double fy = state.y + this->wheel_base * std::sin(state.yaw);

    for (uint32_t i = pind; i < (pind + N_IND_SEARCH); i++) {
        double dx = fx - points[i].x;
        double dy = fy - points[i].y;
        double point_to_distance = dx * dx + dy * dy;
        if (min > point_to_distance) {
            min = point_to_distance;
            target_dx = dx;
            target_dy - dy;
            min_point_index = i;
        }
        if (i >= (points.size() - 1)) {
            break;
        }
    }
    double front_axle_vec_x = -std::cos(state.yaw + M_PI / 2);
    double front_axle_vec_y = -std::sin(state.yaw + M_PI / 2);

    err_front_axel = target_dx * front_axle_vec_x + target_dy * front_axle_vec_y;

    return min_point_index;
}

void pid_steer_control::smooth_yaw(std::vector<Point> &points)
{
    for (uint32_t i = 0; i < points.size() - 1; i++) {
        double diff_yaw = points[i + 1].yaw - points[i].yaw;

        while (diff_yaw >= M_PI_2) {
            points[i + 1].yaw -= M_PI * 2.0;
            diff_yaw = points[i + 1].yaw - points[i].yaw;
        }

        while (diff_yaw <= -M_PI_2) {
            points[i + 1].yaw += M_PI * 2.0;
            diff_yaw = points[i + 1].yaw - points[i].yaw;
        }
    }
}

int pid_steer_control::pid_steering_control(ControlState state, double& steer)
{
    double e = 0.0f;
    int current_target_ind = this->calculate_target_index(state, this->points, this->target_ind, e);

    if (current_target_ind < this->target_ind) {
        current_target_ind = this->target_ind;
    }
    double yaw_error = pi_2_pi(this->points[current_target_ind].yaw - state.yaw);
    double th_e = pi_2_pi(yaw_error * this->steer_kp + (yaw_error - this->steer_pre_e) * this->steer_kd);

    this->path_distance_pid.set_target(e);
    double steer_delta = std::atan2(this->path_distance_pid.calculate(0), state.v);

    steer = th_e + steer_delta;

    steer_pre_e = yaw_error;

    return current_target_ind;
}

bool pid_steer_control::update(double dt) {
    double calculated_steer = 0;
    this->dt = dt;

    if (dt == 0) {
        return false;
    }

    this->target_ind = pid_steering_control(this->state, calculated_steer);
    // PID로 가속도 값 계산
    this->path_accel_pid.set_target(this->points[this->target_ind].speed);
    double calculated_accel = this->path_accel_pid.calculate(this->state.v);

    // state update
    this->state = this->update_state(this->state, calculated_accel, calculated_steer, this->dt);
    double state_to_goal_distance = sqrt(pow(this->goal_state.x - this->state.x, 2) + pow(this->goal_state.y - this->state.y, 2));
    size_t remain_point = get_remain_point();
    if (remain_point == 0) {
        // finish
        return true;
    }

    return false;
}

double pid_steer_control::calculate_error()
{
    double distance1 = 100;
    double distance2 = 100;
    int temp_target_index = this->target_ind - 10;

    if (temp_target_index < 0) {
        temp_target_index = 0;
    }

    int nearest_index = this->calculate_nearest_index(state, this->points, temp_target_index);

    auto cal_diatance_between_line_to_point = [](double x1, double y1, double x2, double y2, double stand_x, double stand_y ) -> double {
        double a = (y1 - y2) / (x1 - x2);
        double b = -1;
        double c = y1 - a * x1;

        return abs(a * stand_x + b * stand_y + c) / sqrt(a*a + 1);
    };

    if (nearest_index != 0) {
        distance1 = cal_diatance_between_line_to_point(this->points[nearest_index].x, this->points[nearest_index].y,
                                           this->points[nearest_index - 1].x, this->points[nearest_index - 1].y,
                                           this->state.x, this->state.y);
    }
    if (nearest_index != (this->points.size() - 1)) {
        distance2 = cal_diatance_between_line_to_point(this->points[nearest_index].x, this->points[nearest_index].y,
                                           this->points[nearest_index + 1].x, this->points[nearest_index + 1].y,
                                           this->state.x, this->state.y);
    }
    return distance1 < distance2 ? distance1 : distance2;
}

ControlState pid_steer_control::update_state(ControlState state, double accel, double steer_delta, double dt)
{
    if (steer_delta > this->max_steer_angle) {
        steer_delta = this->max_steer_angle;
    } else if (steer_delta < -this->max_steer_angle) {
        steer_delta = -this->max_steer_angle;
    }
    state.steer = steer_delta;
    // golfcar position, angle update
    state.x = state.x + state.v * std::cos(state.yaw) * dt;
    state.y = state.y + state.v * std::sin(state.yaw) * dt;
    state.yaw = state.yaw + state.v / this->wheel_base * std::tan(steer_delta) * dt;
    // state.v = state.v + accel * dt;
    state.v = this->points[this->target_ind].speed;

    if (state.v > this->max_speed) {
        state.v = this->max_speed;
    } else if (state.v < -this->max_speed) {
        state.v = -this->max_speed;
    }

    return state;
}
