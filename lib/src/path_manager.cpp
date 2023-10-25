#include "path_manager.h"

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <algorithm>


static const double DEFAULT_MAX_STEER = 25.0 * M_PI / 180.0;     // [rad] 45deg
static const double DEFAULT_MAX_SPEED = 10.0 / 3.6;              // [ms] 10km/h
static const double DEFAULT_WHEEL_BASE = 2.15;                   // 앞 뒤 바퀴 사이 거리 [m]

static const double DEGREE1_RAD = 1.5 * 180 / M_PI;
static const double DEFAULT_STEER_MAX_VELOCITY = 15.0 * M_PI / 180.0; // [rad/s] 7deg/s
static const double THRESHOLD_STEER_DIFF_ANGLE = 3 * M_PI / 180.0; // [rad] 5deg
static const double MAX_STEER_DIFF_ANGLE = 15.0 * M_PI / 180.0; // [rad] 10deg
static const double MAX_TAGET_VALID_ANGLE = 30.0 * M_PI / 180.0; // 화각? 현재 스티어 + yaw 위치에서 이 각도 내에 있는 점을 선택
static const int MAX_STEER_ERROR_LEVEL = 10; // steer error 세분화

static double distance_between_point_and_line(Point point, Point line_point1, Point line_point2)
{
    double a = (line_point1.y - line_point2.y) / (line_point1.x - line_point2.x);
    double c = line_point1.y - a * line_point1.x;
    double b = -1;

    double error_distance = fabsf(a * point.x + b * point.y + c) / sqrtf(a * a + b * b);

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
    int goal_index = points.size() - 1;

    this->init_state = init_state;
    this->points = points;

    if (this->init_state.yaw - this->points[0].yaw >= M_PI) {
        this->init_state.yaw -= 2.0f * M_PI;
    } else if (this->init_state.yaw - this->points[0].yaw <= -M_PI) {
        this->init_state.yaw += 2.0f * M_PI;
    }

    this->goal_state = ControlState(this->points[goal_index].x, this->points[goal_index].y, this->points[goal_index].yaw, 0, this->points[goal_index].speed);

    this->target_ind = this->calculate_target_index(this->init_state, this->points, 0);
    this->set_state(this->init_state);
    this->smooth_yaw(this->points);

}

void path_tracking_controller::add_course(ControlState init_state, std::vector<Point> points)
{
    this->points.insert(end(this->points), begin(points), end(points));

    int goal_index = points.size() - 1;

    this->init_state = init_state;  // init start state
    if (this->init_state.yaw - this->points[0].yaw >= M_PI) {
        this->init_state.yaw -= 2.0 * M_PI;
    } else if (this->init_state.yaw - this->points[0].yaw <= -M_PI) {
        this->init_state.yaw += 2.0 * M_PI;
    }
    this->goal_state = ControlState(this->points[goal_index].x, this->points[goal_index].y, this->points[goal_index].yaw, 0, this->points[goal_index].speed);

    this->target_ind = this->calculate_target_index(this->init_state, this->points, 0);
    this->smooth_yaw(this->points);
}

int path_tracking_controller::calculate_target_index(ControlState state, std::vector<Point> points, int pind)
{
    const int N_IND_SEARCH = 5;
    double min = 10000;
    double min_temp = 10000;
    double min_distance = 0;
    uint32_t min_index = 0;
    uint32_t min_point_index = N_IND_SEARCH + pind;
    uint32_t min_point_index_temp = N_IND_SEARCH + pind;

    for (uint32_t i = pind; i < (pind + N_IND_SEARCH); i++) {
        double dx = points[i].x - state.x;
        double dy = points[i].y - state.y;
        double point_to_distance = dx * dx + dy * dy;
        if (min > point_to_distance) {
            double target_angle = sin(atan2(dy, dx) / 4);
            double cw_angle = sin(pi_2_pi(state.yaw + state.steer - MAX_TAGET_VALID_ANGLE) / 4);
            double ccw_angle = sin(pi_2_pi(state.yaw + state.steer + MAX_TAGET_VALID_ANGLE) / 4);
            bool flag = false;

            if (cw_angle < ccw_angle) {
                if (target_angle < ccw_angle && target_angle > cw_angle) {
                    flag = true;
                }
            } else {
                if (target_angle < ccw_angle || target_angle > cw_angle) {
                    flag = true;
                }
            }
            // std::cout << i << " -> target anlge : " << atan2(dy, dx) * 180 / M_PI <<
            //                   " cw_angle : " << pi_2_pi(state.yaw + state.steer - MAX_TAGET_VALID_ANGLE) * 180 / M_PI <<
            //                   " ccw_angle : " << pi_2_pi(state.yaw + state.steer + MAX_TAGET_VALID_ANGLE) * 180 / M_PI <<
            //                   " steer : " << (state.steer) * 180 / M_PI <<
            //                   " yaw : " << state.yaw * 180 / M_PI << std::endl;
            if (flag) {
                min = point_to_distance;
                min_point_index = i;
            }
        }
        if (min_temp > point_to_distance) {
            min_temp = point_to_distance;
            min_point_index_temp = i;
        }
        if (i >= (points.size() - 1)) {
            break;
        }
    }

    // can not find target
    if (min_point_index >= (N_IND_SEARCH + pind)) {
        if (this->state.steer >= max_steer_angle || this->state.steer <= -max_steer_angle) {
            std::cout << "can not find target" << std::endl;
            return -1;
        } else {
            min_distance = min_temp;
            min_index = min_point_index_temp;
        }
    } else {
        min_distance = min;
        min_index = min_point_index;
    }

    if (min_index != 0) {
        Point current_state = {this->state.x, this->state.y, 0, 0, 0};
        min_distance = distance_between_point_and_line(current_state, this->points[min_index - 1], this->points[min_index]);

    } else {
        min_distance = 0;
    }
    this->distance_error = min_distance;

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
    if (this->target_ind == -1) {
        return true;
    }
    if (calculated_steer > this->max_steer_angle) {
        calculated_steer = this->max_steer_angle;
    } else if (calculated_steer < -this->max_steer_angle) {
        calculated_steer = -this->max_steer_angle;
    }
    velocity_control(this->state, calculated_accel);

    // state update
    this->state = this->update_state(this->state, calculated_accel, calculated_steer, this->dt);
    this->target_steer = calculated_steer;
    this->target_velocity = this->state.v;
    
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
    double max_steer_change_amount = DEFAULT_STEER_MAX_VELOCITY * dt;
    double dsteer = steer_delta - state.steer;
    double target_steer = state.steer;
    if (dsteer > max_steer_change_amount) {
        target_steer += max_steer_change_amount;
    } else if (dsteer < -max_steer_change_amount) {
        target_steer -= max_steer_change_amount;
    } else {
        target_steer = steer_delta;
    }

    if (target_steer > this->max_steer_angle) {
        target_steer = this->max_steer_angle;
    } else if (target_steer < -this->max_steer_angle) {
        target_steer = -this->max_steer_angle;
    }

    int velocity_control_level = (int)(fabsf(state.steer - steer_delta) / THRESHOLD_STEER_DIFF_ANGLE);
    state.v = this->points[this->target_ind].speed - (this->points[this->target_ind].speed - 0.5) * ((double)velocity_control_level / 10);
    // state.v = this->points[this->target_ind].speed;

    state.steer = target_steer;

    // golfcar position, angle update
    state.x = state.x + state.v * std::cos(state.yaw) * dt;
    state.y = state.y + state.v * std::sin(state.yaw) * dt;
    state.yaw = state.yaw + state.v / this->wheel_base * std::tan(state.steer) * dt;


    if (state.v > this->max_speed) {
        state.v = this->max_speed;
    } else if (state.v < 0) {
        state.v = 0;
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
