#include "path_manager.h"

// std
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <algorithm>


static const double DEFAULT_WHEEL_BASE = 2.15;                                  /**< 기본 차량 앞뒤 바퀴 간격[m] */

static const double DEFAULT_MIN_SPEED = 1.0;                                    /**< 기본 최소 주행 속도[m/s] */
static const double DEFAULT_MAX_SPEED = 10.0 / 3.6;                             /**< 기본 최대 주행 속도[m/s] */

static const double DEFAULT_MAX_STEER_ANGLE = 25.0 * PT_M_PI / 180.0;           /**< 기본 최대 조향 각도[rad] */
static const double DEFAULT_MAX_STEER_VELOCITY = 20.0 * PT_M_PI / 180.0;        /**< 기본 최대 조향 각속도[rad/s] */

static const double DEFAULT_MAX_MOVEABLE_RANGE = 30.0 * PT_M_PI / 180.0;        /**< 이동 가능한 방향 범위[rad] */

static const double THRESHOLD_STEER_DIFF_ANGLE = 3 * PT_M_PI / 180.0;           /**< 조향각에 따른 속도 조절을 위한 조향각 레졸루션 단위[rad] */
static const int MAX_STEER_ERROR_LEVEL = 10;                                    /**< steer error 세분화 */

path_tracker::path_tracker()
{
    this->init(DEFAULT_MAX_STEER_ANGLE, DEFAULT_MAX_SPEED, DEFAULT_WHEEL_BASE);
    this->target_index_offset = 2;
}

path_tracker::path_tracker(const double max_steer_angle, const double max_speed, const double wheel_base)
{
    this->init(max_steer_angle, max_speed, wheel_base);
    this->target_index_offset = 2;
}

path_tracker::~path_tracker()
{

}

void path_tracker::init(const double max_steer_angle, const double max_speed, const double wheel_base)
{
    this->max_steer_angle = max_steer_angle;
    this->max_speed = max_speed;
    this->wheel_base = wheel_base;

    this->points.clear();
}

void path_tracker::set_path_points(pt_control_state_t init_state, std::vector<path_point_t> points)
{
    path_point_t first_point = points[0];
    int goal_index = points.size() - 1;

    this->points = points;
    this->smooth_yaw(this->points);

    // 초기 상태 설정
    this->init_state = init_state;
    if (this->init_state.yaw - first_point.yaw >= PT_M_PI) {
        this->init_state.yaw -= 2.0 * PT_M_PI;
    } else if (this->init_state.yaw - first_point.yaw <= -PT_M_PI) {
        this->init_state.yaw += 2.0 * PT_M_PI;
    }

    // 목표 상태 설정
    this->goal_state.x = this->points[goal_index].x;
    this->goal_state.y = this->points[goal_index].y;
    this->goal_state.yaw = this->points[goal_index].yaw;
    this->goal_state.steer = 0.0;
    this->goal_state.v = this->points[goal_index].speed;

    // 목표 경로점 인덱스 계산
    this->target_point_index = this->calculate_target_index(this->init_state, this->points, 0);

    // 현재 상태 갱신
    this->set_state(this->init_state);
}

pt_update_result_t path_tracker::update(double time)
{
    double calculated_steer = 0;
    double calculated_velocity = 0;
    int front_point_index = 0;
    path_point_t front_point;
    size_t remain_point = 0;

    if (time <= 0.0) {
        return PT_UPDATE_RESULT_INVAILED_TIME;
    }

    if (this->updated_time == 0) {
        this->updated_time = time;
        return PT_UPDATE_RESULT_NOT_READY;
    }

    this->dt = time - this->updated_time;
    if (this->dt < 0.0) {
        return PT_UPDATE_RESULT_INVAILED_TIME;
    }
    this->updated_time = time;

    // 예측 상태 갱신
    this->predict_state = this->update_predict_state(this->predict_state, this->dt);

    // 목표 경로점 찾기
    this->target_point_index = this->calculate_target_index(this->predict_state, this->points, this->target_point_index);
    if (this->target_point_index == -1) {
        // 경로를 찾을 수 없는 경우
        return PT_UPDATE_RESULT_NOT_FOUND_TARGET;
    }

    // 앞점 계산
    front_point_index = this->get_front_target_point_index();
    front_point = this->points[front_point_index];

    // 조향각 계산
    calculated_steer = steering_control(this->predict_state, front_point);
    if (calculated_steer > this->max_steer_angle) {
        calculated_steer = this->max_steer_angle;
    } else if (calculated_steer < -this->max_steer_angle) {
        calculated_steer = -this->max_steer_angle;
    }

    // 주행 속도 계산
    calculated_velocity = velocity_control(this->predict_state, front_point);

    // 조향각에 따른 주행 속도 재계산
    calculated_velocity = velocity_control_depend_on_steer_error(this->predict_state, calculated_velocity, calculated_steer);

    // 목표 조향각, 주행 속도 설정
    this->target_steer = calculated_steer;
    this->target_velocity = calculated_velocity;

    // 목표지점 도착 확인
    remain_point = get_remain_point_num();
    if (remain_point == 0) {
        return PT_UPDATE_RESULT_GOAL;
    }

    return PT_UPDATE_RESULT_RUNNING;
}

bool path_tracker::is_moveable_point(pt_control_state_t current, path_point_t target, double range)
{
    bool res = false;
    double target_angle = atan2(target.y - current.y, target.x - current.x);
    double cw_angle = path_tracker::pi_to_pi(current.yaw + current.steer - range);
    double ccw_angle = path_tracker::pi_to_pi(current.yaw + current.steer + range);

    // 계산 편의를 위해 1사분면만 사용
    double scale = 4.0;
    double target_angle_sin = sin(target_angle / scale);
    double cw_angle_sin = sin(cw_angle / scale);
    double ccw_angle_sin = sin(ccw_angle / scale);

    if (cw_angle_sin < ccw_angle_sin) {
        if (target_angle_sin < ccw_angle_sin && target_angle_sin > cw_angle_sin) {
            res = true;
        }
    } else {
        if (target_angle_sin < ccw_angle_sin || target_angle_sin > cw_angle_sin) {
            res = true;
        }
    }

    // std::cout << "cw_angle : " << cw_angle / scale * 180.0 / PT_M_PI
    //           << " ccw_angle : " << ccw_angle / scale * 180.0 / PT_M_PI
    //           << " target_angle : " << target_angle / scale * 180.0 / PT_M_PI << std::endl;

    return res;
}

bool path_tracker::get_steer_at_moveable_point(pt_control_state_t current, path_point_t target, double* steer)
{
    double target_yaw = path_tracker::pi_to_pi(atan2(target.y - current.y, target.x - current.x));
    double target_steer = path_tracker::pi_to_pi(target_yaw - current.yaw);

    // 이동 가능한 목표점인지 확인
    if (!this->is_moveable_point(current, target, DEFAULT_MAX_MOVEABLE_RANGE)) {
        return false;
    }

    // 최대 조향각보다 크면 오류
    if (abs(target_steer) > this->max_steer_angle) {
        return false;
    }

    *steer = target_steer;

    return true;
}

pt_control_state_t path_tracker::update_predict_state(pt_control_state_t state, double dt)
{
    // golfcar position, angle update
    state.yaw = state.yaw + state.v / this->wheel_base * std::tan(state.steer) * dt;
    state.x = state.x + state.v * std::cos(state.yaw) * dt;
    state.y = state.y + state.v * std::sin(state.yaw) * dt;

    return state;
}

int path_tracker::calculate_target_index(pt_control_state_t current_state, std::vector<path_point_t>& points, int start_index)
{
    static const int SEARCH_NUM = 5;
    int max_index = start_index + SEARCH_NUM;
    int target_point_index = -1;
    double min_distance = 10000.0;
    path_point_t current_point = {
        .x = current_state.x,
        .y = current_state.y,
        .yaw = 0.0,
        .k = 0.0,
        .speed = 0.0
    };

    if (max_index > points.size() - 1) {
        max_index = points.size() - 1;
    }

    // 현재 조향각 기준으로 좌/우 30도 내에 가장 가까운 점을 목표로 설정
    for (uint32_t i = start_index; i < max_index; i++) {
        path_point_t target_point = points[i];
        double dx = target_point.x - current_state.x;
        double dy = target_point.y - current_state.y;
        double distance = dx * dx + dy * dy;

        if (distance < min_distance) {
            if (this->is_moveable_point(current_state, target_point, DEFAULT_MAX_MOVEABLE_RANGE)) {
                min_distance = distance;
                target_point_index = i;
            }
        }
    }

    // 현재 조향각 기준으로 못 찾은 경우, 최대 조향각 기준으로 다시 찾기
    if (target_point_index == -1) {
        for (int i = start_index; i < max_index; i++) {
            path_point_t target_point = points[i];
            pt_control_state_t left = current_state;
            pt_control_state_t right = current_state;
            bool left_condition = false;
            bool right_condition = false;

            left.steer = this->max_steer_angle;
            left_condition = this->is_moveable_point(left, target_point, DEFAULT_MAX_MOVEABLE_RANGE);

            right.steer = -this->max_steer_angle;
            right_condition = this->is_moveable_point(right, target_point, DEFAULT_MAX_MOVEABLE_RANGE);

            if (left_condition || right_condition) {
                target_point_index = i;
                break;
            }
        }

        // 목표 점을 못찾음
        if (target_point_index == -1) {
            return -1;
        }
    }

    // 거리 오차 계산
    if (target_point_index != 0) {
        this->distance_error = this->distance_between_point_and_line(current_point, this->points[target_point_index - 1], this->points[target_point_index]);
    } else {
        this->distance_error = this->distance_between_point_and_line(current_point, this->points[target_point_index], this->points[target_point_index + 1]);
    }

    // 방향 오차 계산
    this->yaw_error = this->pi_to_pi(this->points[target_point_index].yaw - current_state.yaw);

    return target_point_index;
}

double path_tracker::velocity_control_depend_on_steer_error(pt_control_state_t state, double target_velocity, double target_steer)
{
    double max_steer_change_amount = DEFAULT_MAX_STEER_VELOCITY * dt;
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

double path_tracker::distance_between_point_and_line(path_point_t point, path_point_t line_point1, path_point_t line_point2)
{
    double error_distance = 0.0;
    double a = 0.0;
    double b = -1.0;
    double c = 0.0;

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

    // 왼쪽(-) 오른쪽(+)
    return error_distance;
}

void path_tracker::smooth_yaw(std::vector<path_point_t> &points)
{
    for (uint32_t i = 0; i < points.size() - 1; i++)
    {
        double diff_yaw = points[i + 1].yaw - points[i].yaw;

        while (diff_yaw >= PT_M_PI_2) {
            points[i + 1].yaw -= PT_M_PI * 2.0;
            diff_yaw = points[i + 1].yaw - points[i].yaw;
        }

        while (diff_yaw <= -PT_M_PI_2) {
            points[i + 1].yaw += PT_M_PI * 2.0;
            diff_yaw = points[i + 1].yaw - points[i].yaw;
        }
    }
}

double path_tracker::pi_to_pi(double angle)
{
    while (angle > PT_M_PI) {
        angle = angle - 2.0 * PT_M_PI;
    }

    while (angle < -PT_M_PI) {
        angle = angle + 2.0 * PT_M_PI;
    }

    return angle;
}
