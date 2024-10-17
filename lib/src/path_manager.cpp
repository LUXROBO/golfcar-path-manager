#include "path_manager.h"

// std
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <algorithm>


static const float DEFAULT_WHEEL_BASE = 2.15;                                  /**< 기본 차량 앞뒤 바퀴 간격[m] */

static const float DEFAULT_MIN_SPEED = 1.0;                                    /**< 기본 최소 주행 속도[m/s] */
static const float DEFAULT_MAX_SPEED = 10.0 / 3.6;                             /**< 기본 최대 주행 속도[m/s] */

static const float DEFAULT_MAX_STEER_ANGLE = 25.0 * PT_M_PI / 180.0;           /**< 기본 최대 조향 각도[rad] */
static const float DEFAULT_MAX_STEER_VELOCITY = 20.0 * PT_M_PI / 180.0;        /**< 기본 최대 조향 각속도[rad/s] */
static const float DEFAULT_MAX_VEHICLE_ACCEL = 0.8333333;                      /**< 기본 최대 주행 가속도[m/s^2] */

static const float DEFAULT_MAX_MOVEABLE_RANGE = 30.0 * PT_M_PI / 180.0;        /**< 이동 가능한 방향 범위[rad] */

static const float THRESHOLD_STEER_DIFF_ANGLE = 3 * PT_M_PI / 180.0;           /**< 조향각에 따른 속도 조절을 위한 조향각 레졸루션 단위[rad] */
static const int MAX_STEER_ERROR_LEVEL = 10;                                    /**< steer error 세분화 */

static const int MAX_LOOK_AHEAD_NUM = 1;

path_tracker::path_tracker()
{
    this->init(DEFAULT_MAX_STEER_ANGLE, DEFAULT_MAX_SPEED, DEFAULT_WHEEL_BASE, 0);
    this->updated_time = 0;
    this->target_index_offset = 3;
    this->lf = DEFAULT_WHEEL_BASE / 2;
    this->lr = DEFAULT_WHEEL_BASE / 2;
}

path_tracker::path_tracker(const float max_steer_angle, const float max_speed, const float wheel_base, const float center_to_gps_distance = 0)
{
    this->init(max_steer_angle, max_speed, wheel_base, center_to_gps_distance);
    this->updated_time = 0;
    this->target_index_offset = 3;
    this->lf = wheel_base / 2 - center_to_gps_distance;
    this->lr = wheel_base / 2 + center_to_gps_distance;
}

path_tracker::~path_tracker()
{

}

void path_tracker::init(const float max_steer_angle, const float max_speed, const float wheel_base, const float center_to_gps_distance = 0)
{
    this->max_steer_angle = max_steer_angle;
    this->max_speed = max_speed;
    this->wheel_base = wheel_base;
    this->lf = wheel_base / 2 - center_to_gps_distance;
    this->lr = wheel_base / 2 + center_to_gps_distance;
    this->points.clear();
}

void path_tracker::set_path_points(pt_control_state_t init_state, std::vector<path_point_t> points)
{
    int start_index = 0;
    int goal_index = 0;

    this->points = points;
    this->smooth_yaw(this->points);

    start_index = 0;
    goal_index = points.size() - 1;

    // 초기 상태 설정
    this->init_state = init_state;
    if (this->init_state.yaw - points[start_index].yaw >= PT_M_PI) {
        this->init_state.yaw -= 2.0 * PT_M_PI;
    } else if (this->init_state.yaw - points[start_index].yaw <= -PT_M_PI) {
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

pt_update_result_t path_tracker::update(float dt)
{
    float calculated_steer = 0;
    float calculated_velocity = 0;
    std::vector<int> look_ahead_index;
    std::vector<path_point_t> look_ahead_point;
    int goal_point_index = 0;
    path_point_t front_point;
    size_t remain_point = 0;

    if (dt <= 0.0) {
        return PT_UPDATE_RESULT_INVAILED_TIME;
    }

    this->dt = dt;

    // 목표 경로점 찾기
    this->target_point_index = this->calculate_target_index(this->state, this->points, this->target_point_index);
    if (this->target_point_index == -1) {
        // 경로를 찾을 수 없는 경우
        return PT_UPDATE_RESULT_NOT_FOUND_TARGET;
    }

    // 앞점 계산
    int start_index = this->get_front_target_point_index();
    look_ahead_index.push_back(start_index);
    look_ahead_point.push_back(points[start_index]);
    for (int i = 0; i < MAX_LOOK_AHEAD_NUM - 1; i++) {
        look_ahead_index.push_back(this->get_front_target_point_index(start_index));
        look_ahead_point.push_back(points[look_ahead_index[i]]);
        start_index = look_ahead_index[i];
    }

    // 도착 경로점 계산
    goal_point_index = this->points.size() - 1;

    // 조향각 계산
    calculated_steer = steering_control(this->state, look_ahead_point);
    if (calculated_steer > this->max_steer_angle) {
        calculated_steer = this->max_steer_angle;
    } else if (calculated_steer < -this->max_steer_angle) {
        calculated_steer = -this->max_steer_angle;
    }

    // 주행 속도 계산
    calculated_velocity = velocity_control(this->state, front_point);

    // 조향각에 따른 주행 속도 재계산
    calculated_velocity = velocity_control_depend_on_steer_error(this->state, calculated_velocity, calculated_steer);

    // 목표 조향각, 주행 속도 설정
    this->target_steer = calculated_steer;// * 0.5 + this->target_steer * 0.5;
    this->target_velocity = calculated_velocity;

    // 목표지점 도착 확인
    remain_point = get_remain_point_num();
    if ((remain_point == 0) || (look_ahead_index[0] == goal_point_index)) {
        this->target_velocity = 0.0;
        return PT_UPDATE_RESULT_GOAL;
    }

    return PT_UPDATE_RESULT_RUNNING;
}

bool path_tracker::is_moveable_point(pt_control_state_t current, path_point_t target, float range)
{
    bool res = false;
    float target_angle = atan2(target.y - current.y, target.x - current.x);
    float cw_angle = path_tracker::pi_to_pi(current.yaw + current.steer - range);
    float ccw_angle = path_tracker::pi_to_pi(current.yaw + current.steer + range);

    // 계산 편의를 위해 1사분면만 사용
    float scale = 4.0;
    float target_angle_sin = sin(target_angle / scale);
    float cw_angle_sin = sin(cw_angle / scale);
    float ccw_angle_sin = sin(ccw_angle / scale);

    if (cw_angle_sin < ccw_angle_sin) {
        if (target_angle_sin < ccw_angle_sin && target_angle_sin > cw_angle_sin) {
            res = true;
        }
    } else {
        if (target_angle_sin < ccw_angle_sin || target_angle_sin > cw_angle_sin) {
            res = true;
        }
    }
    return res;
}

bool path_tracker::get_steer_at_moveable_point(pt_control_state_t current, path_point_t target, float* steer)
{
    float target_yaw = path_tracker::pi_to_pi(atan2(target.y - current.y, target.x - current.x));
    float target_steer = path_tracker::pi_to_pi(target_yaw - current.yaw);

    // 이동 가능한 목표점인지 확인
    if (!this->is_moveable_point(current, target, DEFAULT_MAX_MOVEABLE_RANGE)) {
        return false;
    }

    // 최대 조향각보다 크면 오류
    if (fabsf(target_steer) > this->max_steer_angle) {
        return false;
    }

    *steer = target_steer;

    return true;
}

path_point_t path_tracker::get_point_cross_two_line(path_point_t point1, float slope1, path_point_t point2, float slope2)
{
    float b1 = point1.y - slope1 * point1.x;
    float b2 = point2.y - slope2 * point2.x;

    float x3 = (b2 - b1) / (slope1 - slope2);
    float y3 = slope1 * x3 + b1;

    return path_point_t{x3, y3, 0, 0, 0};
}

path_point_t path_tracker::get_path_circle(path_point_t point1, path_point_t point2, float slope)
{
    // double orthogonal_yaw = path_tracker::pi_to_pi(yaw + PT_M_PI_2);
    double x = 0;
    double y = 0;
    double xx1 = pow(point1.x, 2);
    double yy1 = pow(point1.y, 2);
    double xx2 = pow(point2.x, 2);
    double yy2 = pow(point2.y, 2);

    if (fabsf(slope) > 10000) {
        x = point1.x;
        y = (yy1 - pow(point2.x - point1.x, 2) - yy2) / (2 * (point1.y - point2.y));
    } else if (fabsf(slope) < 0.001){
        y = point1.y;
        x = (xx1 - pow(point2.y - point1.y, 2) - xx2) / (2 * (point1.x - point2.x));
    } else {
        float a = slope;
        float b = point1.y - a * point1.x;
        x = (xx1 - 2 * b * point1.y + yy1 - xx2 + 2 * b * point2.y - yy2) / (2 * (point1.x + a * point1.y - point2.x - a * point2.y));
        y = a * x + b;
    }
    float distance = sqrt(pow(x - point1.x, 2) + pow(y - point1.y, 2));

    return path_point_t{x, y, 0, 1 / distance, 0};
}

pt_control_state_t path_tracker::update_predict_state(pt_control_state_t state, float dt)
{
    path_point_t front_wheel_point = {state.x + this->lf * std::cos(state.yaw), state.y + this->lf * std::sin(state.yaw), 0, 0, 0};
    path_point_t rear_wheel_point = {state.x - this->lr * std::cos(state.yaw), state.y - this->lr * std::sin(state.yaw), 0, 0, 0};

    if (state.steer == 0) {
        state.x += state.v * dt * std::cos(state.yaw);
        state.y += state.v * dt * std::sin(state.yaw);
        return state;
    }

    path_point_t rotation_origin_point = this->get_point_cross_two_line(rear_wheel_point, std::tan(path_tracker::pi_to_pi(state.yaw + PT_M_PI_2)),
                                                                               front_wheel_point, std::tan(path_tracker::pi_to_pi(state.yaw + state.steer + PT_M_PI_2)));
    float center_slope = path_tracker::pi_to_pi(std::atan2(rotation_origin_point.y - state.y, rotation_origin_point.x - state.x));

    if (fabsf(path_tracker::pi_to_pi(center_slope + PT_M_PI_2 - state.yaw)) < fabsf(path_tracker::pi_to_pi(center_slope - PT_M_PI_2 - state.yaw))) {
        center_slope += PT_M_PI_2;
    } else {
        center_slope -= PT_M_PI_2;
    }

    float center_slip_angle = path_tracker::pi_to_pi(center_slope - state.yaw);
    this->g_vl = state.v * std::cos(center_slip_angle);
    this->g_vr = state.v * std::sin(center_slip_angle);

    state.x += state.v * dt * std::cos(state.yaw + center_slip_angle);
    state.y += state.v * dt * std::sin(state.yaw + center_slip_angle);
    state.yaw += this->g_vl * dt * std::tan(center_slip_angle) / this->wheel_base;
    state.yaw = path_tracker::pi_to_pi(state.yaw);

    return state;
}

int path_tracker::calculate_target_index(pt_control_state_t current_state, std::vector<path_point_t>& points, int start_index)
{
    static const int SEARCH_NUM = 5;
    int max_index = start_index + SEARCH_NUM;
    int target_point_index = -1;
    float min_distance = 10000.0;
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
        float dx = target_point.x - current_state.x;
        float dy = target_point.y - current_state.y;
        float distance = dx * dx + dy * dy;

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
        this->distance_error = this->get_line_distance(current_point, this->points[target_point_index - 1], this->points[target_point_index]);
    } else {
        this->distance_error = this->get_line_distance(current_point, this->points[target_point_index], this->points[target_point_index + 1]);
    }

    // 방향 오차 계산
    this->yaw_error = this->pi_to_pi(this->points[target_point_index].yaw - current_state.yaw);

    return target_point_index;
}

float path_tracker::velocity_control_depend_on_steer_error(pt_control_state_t state, float target_velocity, float target_steer)
{
    float max_steer_change_amount = DEFAULT_MAX_STEER_VELOCITY * this->dt;
    float dsteer = target_steer - state.steer;
    float revise_target_steer = state.steer;
    float calculated_velocity = 0;
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
    calculated_velocity = target_velocity - (target_velocity - DEFAULT_MIN_SPEED) * ((float)velocity_control_level / MAX_STEER_ERROR_LEVEL);

    if (calculated_velocity > this->max_speed) {
        calculated_velocity = this->max_speed;
    } else if (calculated_velocity < DEFAULT_MIN_SPEED) {
        calculated_velocity = DEFAULT_MIN_SPEED;
    }

    float max_velocity_change_amount = DEFAULT_MAX_VEHICLE_ACCEL * this->dt;
    float d_velocity = calculated_velocity - state.v;
    float predict_velocity = state.v;
    if (d_velocity > max_velocity_change_amount) {
        predict_velocity += max_velocity_change_amount;
    } else if (d_velocity < -max_velocity_change_amount) {
        predict_velocity -= max_velocity_change_amount;
    } else {
        predict_velocity  = calculated_velocity;
    }

    this->revise_target_steer = revise_target_steer;
    // this->state.v = predict_velocity;

    return calculated_velocity;
}

float path_tracker::get_point_distance(path_point_t current_point, path_point_t point)
{
    double x = point.x - current_point.x;
    double y = point.y - current_point.y;
    float distance = sqrt(x * x + y * y);

    return distance;
}

float path_tracker::get_line_distance(path_point_t current_point, path_point_t line_point1, path_point_t line_point2)
{
    float distance = 0.0;
    float a = 0.0;
    float b = -1.0;
    float c = 0.0;

    if (line_point1.x == line_point2.x) {
        distance = (current_point.x - line_point1.x);

        // 진행 방향 아래쪽
        if (line_point1.y > line_point2.y) {
            distance *= -1;
        }

        return distance;
    }

    a = (line_point1.y - line_point2.y) / (line_point1.x - line_point2.x);
    c = line_point1.y - a * line_point1.x;

    distance = fabsf(a * current_point.x + b * current_point.y + c) / sqrt(a * a + b * b);

    if (current_point.y > (a * current_point.x + c)) {
        if (line_point2.x > line_point1.x) {
            distance *= -1;
        }
    } else {
        if (line_point2.x < line_point1.x) {
            distance *= -1;
        }
    }

    // 왼쪽(-) 오른쪽(+)
    return distance;
}

void path_tracker::smooth_yaw(std::vector<path_point_t> &points)
{
    for (uint32_t i = 0; i < points.size() - 1; i++)
    {
        float diff_yaw = points[i + 1].yaw - points[i].yaw;

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

float path_tracker::pi_to_pi(float angle)
{
    while (angle > PT_M_PI) {
        angle = angle - 2.0 * PT_M_PI;
    }

    while (angle < -PT_M_PI) {
        angle = angle + 2.0 * PT_M_PI;
    }

    return angle;
}
