#include "path_manager.h"

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <algorithm>


static const double DEFAULT_MAX_STEER = 45.0 * M_PI / 180.0;     // [rad] 45deg
static const double DEFAULT_MAX_SPEED = 10.0 / 3.6;              // [ms] 10km/h
static const double DEFAULT_WHEEL_BASE = 2.15;                   // 앞 뒤 바퀴 사이 거리 [m]

static double distance_between_point_and_line(Point point, Point line_point1, Point line_point2)
{
    double a = (line_point1.y - line_point2.y) / (line_point1.x - line_point2.x);
    double c = line_point1.y - a * line_point1.x;
    double b = -1;

    double error_distance = abs(a * point.x + b * point.y + c) / sqrt(a * a + b * b);

    if (point.y > (a * point.x + c)) {
        if (line_point2.x > line_point1.x) {
            error_distance *= -1;
        }
    } else {
        if (line_point2.x < line_point1.x) {
            error_distance *= -1;
        }
    }
    return error_distance;
}


path_tracking_controller::~path_tracking_controller()
{

}

path_tracking_controller::path_tracking_controller()
{
    this->max_steer_angle = DEFAULT_MAX_STEER;
    this->max_speed = DEFAULT_MAX_SPEED;
    this->wheel_base = DEFAULT_WHEEL_BASE;
}

path_tracking_controller::path_tracking_controller(const double max_steer_angle, const double max_speed, const double wheel_base)
{
    this->max_steer_angle = max_steer_angle;
    this->max_speed = max_speed;
    this->wheel_base = wheel_base;
}

void path_tracking_controller::init(const double max_steer_angle, const double max_speed, const double wheel_base)
{
    this->points.clear();

    this->max_steer_angle = max_steer_angle;
    this->max_speed = max_speed;
    this->wheel_base = wheel_base;
}

void path_tracking_controller::set_course(ControlState init_state, std::vector<Point> points)
{
	double distance_error = 0;
    int goal_index = points.size() - 1;

    this->init_state = init_state;
    this->points = points;

    if (this->init_state.yaw - this->points[0].yaw >= M_PI) {
        this->init_state.yaw -= 2.0f * M_PI;
    } else if (this->init_state.yaw - this->points[0].yaw <= -M_PI) {
        this->init_state.yaw += 2.0f * M_PI;
    }

    this->goal_state = ControlState(this->points[goal_index].x, this->points[goal_index].y, this->points[goal_index].yaw, 0, this->points[goal_index].speed);

    this->target_ind = this->calculate_target_index(this->init_state, this->points, 0, distance_error);
    this->smooth_yaw(this->points);
}

void path_tracking_controller::add_course(ControlState init_state, std::vector<Point> points)
{
    double distance_error =0;
    this->points.insert(end(this->points), begin(points), end(points));

    int goal_index = points.size() - 1;

    this->init_state = init_state;  // init start state
    if (this->init_state.yaw - this->points[0].yaw >= M_PI) {
        this->init_state.yaw -= 2.0 * M_PI;
    } else if (this->init_state.yaw - this->points[0].yaw <= -M_PI) {
        this->init_state.yaw += 2.0 * M_PI;
    }
    this->goal_state = ControlState(this->points[goal_index].x, this->points[goal_index].y, this->points[goal_index].yaw, 0, this->points[goal_index].speed);

    this->target_ind = this->calculate_target_index(this->init_state, this->points, 0, distance_error);
    this->smooth_yaw(this->points);
    this->points.back().speed = 2.2;
}

int path_tracking_controller::calculate_target_index(ControlState state, std::vector<Point> points, int pind, double& min_distance_ref)
{
    const int N_IND_SEARCH = 5;
    double min = 10000;
    uint32_t min_point_index = 0;

    double fx = state.x + this->wheel_base * std::cos(state.yaw);
    double fy = state.y + this->wheel_base * std::sin(state.yaw);

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

    double min_distance = min;
    uint32_t min_index = min_point_index;

    if (min_index != 0) {
        Point current_state = {this->state.x, this->state.y, 0, 0, 0};
        min_distance = distance_between_point_and_line(current_state, this->points[min_index - 1], this->points[min_index]);

    } else {
        min_distance = 0;
    }
    min_distance_ref = min_distance;

    return min_index;
}

void path_tracking_controller::smooth_yaw(std::vector<Point> &points)
{
    for (uint32_t i = 0; i < points.size() - 1; i++)
    {
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

bool path_tracking_controller::update(double dt) {
    // dt 저장

    double calculated_steer = 0;
    double calculated_accel = 0;
    this->dt = dt;
    if (dt == 0) {
        return false;
    }

    this->target_ind = steering_control(this->state, calculated_steer);
    velocity_control(this->state, calculated_accel);

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

ControlState path_tracking_controller::update_state(ControlState state, double accel, double steer_delta, double dt)
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
    state.v = this->points[this->target_ind].speed;

    if (state.v > this->max_speed) {
        state.v = this->max_speed;
    } else if (state.v < -this->max_speed) {
        state.v = -this->max_speed;
    }

    return state;
}

double path_tracking_controller::pi_2_pi(double angle)
{
    while (angle > M_PI) {
        angle = angle - 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle = angle + 2.0 * M_PI;
    }
    return angle;
}


int path_tracking_controller::steering_control(ControlState state, double& steer)
{
    std::cout << "steering no init" << std::endl;
    return steer;
}

int path_tracking_controller::velocity_control(ControlState state, double& accel)
{
    return accel;
}

void path_tracking_controller::get_gain(int gain_index, double* gain_value)
{

}

void path_tracking_controller::set_gain(int gain_index, double* gain_value)
{

}
