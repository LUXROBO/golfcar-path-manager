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
static const float DEFAULT_CURVATURE_LOW_PASS_FILTER_TAU = 0.35;
static const float MAX_CURVATURE_LOW_PASS_FILTER_TAU = 0.4;

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

float debug_p_gain = 0.8f;

float curvature_steer_control::steering_control(pt_control_state_t state, std::vector<path_point_t> target_point, uint8_t mode)
{
    path_point_t current_state_to_point = {state.x, state.y, state.yaw, 0, 0};
    std::vector<path_point_t> circle_paths;
    float new_yaw = path_tracker::pi_to_pi(state.yaw);
    float w = 0;
    float output;
    float target_curvature = 0;
    float lpf_tau = DEFAULT_CURVATURE_LOW_PASS_FILTER_TAU;
    float new_p_gain = this->yaw_kp; // debugging을 위해 i gain을 사용 중이나 p로 변경 필요
    static float past_curvature = 0;
    static float past_distance_error = 0;
    int used_size = target_point.size();

    // 입력 받은 목표 점들로 이동하기 위한 곡선 경도들의 곡률값의 평균을 예산
    for (int i = 0; i < target_point.size(); i++) {
        path_point_t circle_path = path_tracker::get_path_circle(current_state_to_point, target_point[i], tan(path_tracker::pi_to_pi(new_yaw + PT_M_PI_2)));
        // 계산된 곡률 기반으로 방향을 계산
        float circle_x_for_direction = circle_path.x - state.x;
        float circle_y_for_direction = circle_path.y - state.y;

        float rotation_y = std::sin(-state.yaw) * circle_x_for_direction + std::cos(-state.yaw) * circle_y_for_direction;
        if (rotation_y < 0) {
            circle_path.k *= -1;
        }
        target_curvature += circle_path.k;
    }
    target_curvature /= used_size;

    // 조향 = atan(곡률 * W)
    // 현재 차량 중앙 기준 거리 값이 더 적확
    target_curvature = std::atan(target_curvature * 1.29);

    if (mode != 0) {
        if (mode == 2) {
            target_curvature = target_curvature * this->yaw_ki;
        }
    } else {
        // 특별 구간이 아닌 경우 거리 오차 반영 x
        new_p_gain = 0;
    }

    target_curvature = target_curvature * lpf_tau + past_curvature * (1 - lpf_tau);

    // 타겟 조향 각도와 현재 조향각 에러 값을 통한 pid 계산
    float error = target_curvature - this->state.steer;
    output = this->state.steer + error * 0.5;

    // 거리 에러 적용 @Todo gain 변수 이름이 혼동
    output += this->distance_error * new_p_gain + (this->distance_error - past_distance_error) * this->yaw_kd;

    past_distance_error = this->distance_error;

    this->yaw_pre_e = error;
    past_curvature = target_curvature;

    return output;
}

float curvature_steer_control::velocity_control(pt_control_state_t state, path_point_t target_point)
{
    return target_point.speed;
}
