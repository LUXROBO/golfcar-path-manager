#include "path_manager.h"

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <algorithm>


static const double DEFAULT_MAX_STEER = 25.0 * M_PI / 180.0;     // [rad] 45deg
static const double DEFAULT_MAX_SPEED = 10.0 / 3.6;              // [ms] 10km/h
static const double DEFAULT_WHEEL_BASE = 2.15;                   // 앞 뒤 바퀴 사이 거리 [m]
static const double DEFAULT_MIN_SPEED = 1;                     // [ms]

static const double DEGREE1_RAD = 1.5 * 180 / M_PI;
static const double DEFAULT_STEER_MAX_VELOCITY = 20.0 * M_PI / 180.0; // 조향각 최대 각속도 [rad/s]
static const double THRESHOLD_STEER_DIFF_ANGLE = 3 * M_PI / 180.0; // 조향각에 따른 속도 조절을 위한 조향각 레졸루션 단위[rad] 5deg
static const double MAX_STEER_DIFF_ANGLE = 15.0 * M_PI / 180.0; // 조향각에 따른 속도 조절을 위한 조향각 최대 오차(이 시점 이후론 최저 속도로 주행)[rad] 10deg
static const double MAX_TAGET_VALID_ANGLE = 30.0 * M_PI / 180.0; // 화각? 현재 스티어 + yaw 위치에서 이 각도 내에 있는 점을 선택
static const int MAX_STEER_ERROR_LEVEL = 10; // steer error 세분화

// 왼쪽 -, 오른쪽 +
double path_tracking_controller::distance_between_point_and_line(Point point, Point line_point1, Point line_point2)
{
    double error_distance = 0;
    double a = 0;
    double b = -1;
    double c = 0;

    if (line_point1.x == line_point2.x) {
        error_distance = (point.x - line_point1.x);

        // 진행 방향 아래쪽
        if (line_point1.y > line_point2.y) {
            error_distance *= -1;
        }

        return error_distance;
    }

    a = (line_point1.y - line_point2.y) / (line_point1.x - line_point2.x);
    c = line_point1.y - a * line_point1.x;

    error_distance = abs(a * point.x + b * point.y + c) / sqrt(a * a + b * b);

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

    this->jumping_point = 2;
}

path_tracking_controller::path_tracking_controller(const double max_steer_angle, const double max_speed, const double wheel_base)
{
    this->max_steer_angle = max_steer_angle;
    this->max_speed = max_speed;
    this->wheel_base = wheel_base;

    this->jumping_point = 1;
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
    this->set_state(this->init_state);
    this->smooth_yaw(this->points);
}

bool path_tracking_controller::is_point_in_correct_range(double dx, double dy, double yaw, double steer, double range_angle)
{
    double target_angle = sin(atan2(dy, dx) / 4);
    double cw_angle = sin(pi_2_pi(yaw + steer - range_angle) / 4);
    double ccw_angle = sin(pi_2_pi(yaw + steer + range_angle) / 4);
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
    // std::cout << "cw_angle : " << pi_2_pi(yaw + steer - range_angle) / 4 * 180 / M_PI
    //           << " ccw_angle : " << pi_2_pi(yaw + steer + range_angle) / 4 * 180 / M_PI
    //           << " target_angle : " << atan2(dy, dx) / 4 * 180 / M_PI << std::endl;
    return flag;
}

int path_tracking_controller::calculate_target_index(ControlState state, std::vector<Point> points, int pind)
{
    const int N_IND_SEARCH = 5;
    double min = 10000;
    int min_index = -1;
    uint32_t min_point_index = N_IND_SEARCH + pind;

    for (uint32_t i = pind; i < (pind + N_IND_SEARCH); i++) {
        double dx = points[i].x - state.x;
        double dy = points[i].y - state.y;
        double point_to_distance = dx * dx + dy * dy;
        if (min > point_to_distance) {
            // 현재 steer를 기준으로 좌 우 30도 내에 가장 가까운 점을 타겟으로 설정
            if (is_point_in_correct_range(dx, dy, state.yaw, state.steer, MAX_TAGET_VALID_ANGLE)) {
                min = point_to_distance;
                min_point_index = i;
            }
        }
        if (i >= (points.size() - 1)) {
            break;
        }
    }

    // can not find target
    if (min_point_index >= (N_IND_SEARCH + pind)) {
        for (int i = 0; i < N_IND_SEARCH; i++) {
            double dx = points[i + pind].x - state.x;
            double dy = points[i + pind].y - state.y;
            if (is_point_in_correct_range(dx, dy, state.yaw, this->max_steer_angle, MAX_TAGET_VALID_ANGLE) ||
                is_point_in_correct_range(dx, dy, state.yaw, -this->max_steer_angle, MAX_TAGET_VALID_ANGLE)) {
                min_index = i + pind;
                break;
            }
        }
        if (min_index == -1) {
            // std::cout << "can not find target" << std::endl;
            return -1;
        }
    } else {
        min_index = min_point_index;
    }

    Point current_state = {this->state.x, this->state.y, 0, 0, 0};
    if (min_index != 0) {
        this->distance_error = distance_between_point_and_line(current_state, this->points[min_index - 1], this->points[min_index]);
    } else {
        this->distance_error = distance_between_point_and_line(current_state, this->points[min_index], this->points[min_index + 1]);
    }

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

path_tracking_update_result_t path_tracking_controller::update(double time) {
    // dt 저장

    double calculated_steer = 0;
    double calculated_velocity = 0;
    int jumped_point = 0;
    if (time == 0) {
        return INVAILED_TIME;
    }
    if (updated_time == 0) {
        updated_time = time;
        return NOT_READY;
    }
    this->dt = (time - updated_time) / 1000;

    this->predict_state = this->update_state_for_predict(this->predict_state, dt);

    this->target_ind = this->calculate_target_index(this->predict_state, this->points, this->target_ind);
    if (this->target_ind == -1) {
        // 경로를 찾을 수 없는 경우
        return OUT_OF_RANGE;
    }
    jumped_point = this->target_ind + this->jumping_point;
    if (jumped_point >= this->points.size()) {
        jumped_point = this->target_ind;
    }

    calculated_steer = steering_control(this->predict_state, this->points[jumped_point]);
    if (calculated_steer > this->max_steer_angle) {
        calculated_steer = this->max_steer_angle;
    } else if (calculated_steer < -this->max_steer_angle) {
        calculated_steer = -this->max_steer_angle;
    }
    calculated_velocity = velocity_control(this->predict_state, this->points[jumped_point]);
    calculated_velocity = velocity_control_depend_on_steer_error(this->predict_state, calculated_velocity, calculated_steer);

    this->target_steer = calculated_steer;
    this->target_velocity = calculated_velocity;

    size_t remain_point = get_remain_point();

    updated_time = time;

    if (remain_point == 0) {
        // finish
        return GOAL;
    }
    return RUNNING;
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

    int velocity_control_level = (int)(fabsf(dsteer) / THRESHOLD_STEER_DIFF_ANGLE);
    state.v = this->points[this->target_ind].speed - (this->points[this->target_ind].speed - 0.5) * ((double)velocity_control_level / MAX_STEER_ERROR_LEVEL);

    state.steer = target_steer;

    // golfcar position, angle update
    state.x = state.x + state.v * std::cos(state.yaw) * dt;
    state.y = state.y + state.v * std::sin(state.yaw) * dt;
    state.yaw = state.yaw + state.v / this->wheel_base * std::tan(state.steer) * dt;

    if (state.v > this->max_speed) {
        state.v = this->max_speed;
    } else if (state.v < DEFAULT_MIN_SPEED && state.v > 0) {
        state.v = DEFAULT_MIN_SPEED;
    }

    return state;
}

double path_tracking_controller::velocity_control_depend_on_steer_error(ControlState state, double target_velocity, double target_steer)
{
    double max_steer_change_amount = DEFAULT_STEER_MAX_VELOCITY * dt;
    double dsteer = target_steer - state.steer;
    double revise_target_steer = state.steer;
    double calculated_velocity = 0;
    if (dsteer > max_steer_change_amount) {
        revise_target_steer += max_steer_change_amount;
    } else if (dsteer < -max_steer_change_amount) {
        revise_target_steer -= max_steer_change_amount;
    } else {
        revise_target_steer = target_steer;
    }

    if (revise_target_steer > this->max_steer_angle) {
        revise_target_steer = this->max_steer_angle;
    } else if (revise_target_steer < -this->max_steer_angle) {
        revise_target_steer = -this->max_steer_angle;
    }

    int velocity_control_level = (int)(fabsf(dsteer) / THRESHOLD_STEER_DIFF_ANGLE);
    calculated_velocity = target_velocity - (target_velocity - DEFAULT_MIN_SPEED) * ((double)velocity_control_level / MAX_STEER_ERROR_LEVEL);

    if (calculated_velocity > this->max_speed) {
        calculated_velocity = this->max_speed;
    } else if (calculated_velocity < DEFAULT_MIN_SPEED && calculated_velocity > 0) {
        calculated_velocity = DEFAULT_MIN_SPEED;
    }

    this->predict_state.steer = revise_target_steer;
    this->predict_state.v = calculated_velocity;

    return calculated_velocity;
}

ControlState path_tracking_controller::update_state_for_predict(ControlState state, double dt)
{
    // golfcar position, angle update
    state.yaw = state.yaw + state.v / this->wheel_base * std::tan(state.steer) * dt;
    state.x = state.x + state.v * std::cos(state.yaw) * dt;
    state.y = state.y + state.v * std::sin(state.yaw) * dt;

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

bool path_tracking_controller::get_target_steer_at(Point point, double* steer)
{
    double target_yaw = pi_2_pi(atan2(point.y - this->state.y, point.x - this->state.x));
    double target_steer = pi_2_pi(target_yaw - this->state.yaw);

    if (abs(target_steer) > this->max_steer_angle) {
        return false;
    }

    *steer = target_steer;

    return true;
}

double path_tracking_controller::steering_control(ControlState state, Point target_point)
{
    // std::cout << "steering no init" << std::endl;
    return 0;
}

double path_tracking_controller::velocity_control(ControlState state, Point target_point)
{
    return 0;
}

void path_tracking_controller::get_gain(int gain_index, double* gain_value)
{

}

void path_tracking_controller::set_gain(int gain_index, double* gain_value)
{

}
