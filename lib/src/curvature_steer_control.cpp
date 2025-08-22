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

float curvature_steer_control::constrained_steering_control(pt_control_state_t state, std::vector<path_point_t> target_point, uint8_t mode, float dt)
{
       // ===== [1] 기존 steering_control 로직 (계산 부분 그대로 유지) =====
    path_point_t current_state_to_point = {state.x, state.y, state.yaw, 0, 0};
    float new_yaw = path_tracker::pi_to_pi(state.yaw);
    float target_curvature = 0;
    float lpf_tau = DEFAULT_CURVATURE_LOW_PASS_FILTER_TAU;
    static float past_curvature = 0;
    static float past_distance_error = 0;

    // 곡률 평균 계산
    for (int i = 0; i < target_point.size(); i++) {
        path_point_t circle_path = path_tracker::get_path_circle(
            current_state_to_point,
            target_point[i],
            tan(path_tracker::pi_to_pi(new_yaw + PT_M_PI_2))
        );

        float circle_x = circle_path.x - state.x;
        float circle_y = circle_path.y - state.y;
        float rot_y = std::sin(-state.yaw) * circle_x + std::cos(-state.yaw) * circle_y;
        if (rot_y < 0) circle_path.k *= -1;
        target_curvature += circle_path.k;
    }
    if (!target_point.empty()) target_curvature /= target_point.size();

    // 조향 기본 출력 (기존 계산식)
    float curvature_cal_val = 0;
    {
        float max_gain = 1.5f;
        float min_gain = 1.25f;
        float v1 = 1.25f;
        float v2 = 2.1f;
        float a = (max_gain - min_gain) / (v1 - v2);
        float b = max_gain - a * v1;
        curvature_cal_val = a * state.v + b;
        if (curvature_cal_val < min_gain) curvature_cal_val = min_gain;
    }

    target_curvature = std::atan(target_curvature * curvature_cal_val);
    target_curvature = target_curvature * lpf_tau + past_curvature * (1 - lpf_tau);

    float error = target_curvature - this->state.steer;
    float output = this->state.steer + error * 0.65f;
    output += this->distance_error * this->yaw_kp
           + (this->distance_error - past_distance_error) * this->yaw_kd;

    past_distance_error = this->distance_error;
    this->yaw_pre_e = error;
    past_curvature = target_curvature;

    // ===== [2] 추가된 속도 기반 제약 (새 버전의 핵심) =====

    static float prev_delta    = this->state.steer;
    static float prev_deltaDot = 0.0f;

    const float v  = std::fabs(state.v);

    // δ̇ 상한 스케줄 (rad/s)
    auto rate_limit = [&](float spd)->float {
        if (spd <  LOW_DRIVE_SPEED)  return 1.20f;
        if (spd <  SLOW_DRIVE_SPEED) return 0.90f;
        if (spd <  MID_DRIVE_SPEED)  return 0.55f;
        if (spd <  HIGH_DRIVE_SPEED) return 0.35f;
        return 0.25f;
    };
    float ddelta_max = rate_limit(v);

    // [A] Rate limit
    const float hi = prev_delta + ddelta_max * dt;
    const float lo = prev_delta - ddelta_max * dt;
    float delta_cmd = clamp(output, lo, hi);

    // [B] Jerk limit
    const bool strong_limit = (v >= MID_DRIVE_SPEED);
    const float j_max = strong_limit ? 2.0f : 4.0f; // rad/s^2
    float deltaDot_des = (delta_cmd - prev_delta) / dt;
    float deltaDot_hi  = prev_deltaDot + j_max * dt;
    float deltaDot_lo  = prev_deltaDot - j_max * dt;
    float deltaDot_limited = clamp(deltaDot_des, deltaDot_lo, deltaDot_hi);
    float delta_cmd_jerk = prev_delta + deltaDot_limited * dt;

    // [C] Absolute steering angle limit (측방가속 기준만 적용)
    // 기계적 제한은 외부에서 처리한다고 가정
    const float lat_accel_limit = 2.5; // m/s^2
    const float L = (this->wheel_base > 1e-6f) ? this->wheel_base : 2.0f;
    const float ay_max = (lat_accel_limit > 1e-6f) ? lat_accel_limit : 3.0f;
    const float v2 = MAX(v * v, 1e-3f);

    const float delta_ay = std::atan((ay_max * L) / v2);
    float delta_cmd_bounded = clamp(delta_cmd_jerk, -delta_ay, +delta_ay);

    // [D] 저속 smoothing (MID 미만)
    if (!strong_limit) {
        const float tau = 0.08f; // s
        const float alpha = dt / (tau + dt);
        delta_cmd_bounded = alpha * delta_cmd_bounded + (1.0f - alpha) * prev_delta;
    }

    // 상태 업데이트
    prev_delta    = delta_cmd_bounded;
    prev_deltaDot = (delta_cmd_bounded - prev_delta) / dt;

    return delta_cmd_bounded;
}

float curvature_steer_control::steering_control(pt_control_state_t state, std::vector<path_point_t> target_point, uint8_t mode)
{
    path_point_t current_state_to_point = {state.x, state.y, state.yaw, 0, 0};
    std::vector<path_point_t> circle_paths;
    float new_yaw = path_tracker::pi_to_pi(state.yaw);
    float output;
    float target_curvature = 0;
    float lpf_tau = DEFAULT_CURVATURE_LOW_PASS_FILTER_TAU;
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

    // 조향 = atan(곡률 * W / (v * k))
    float curvature_cal_val = 0;
    float max_curvature_gain = 1.5;
    float min_curvature_gain = 1.25;
    float velocity_for_max_curvature_velocity = 1.25;
    float velocity_for_min_curvature_gain = 2.1;

    float a = (max_curvature_gain - min_curvature_gain) / (velocity_for_max_curvature_velocity - velocity_for_min_curvature_gain);
    float b = max_curvature_gain - a * velocity_for_max_curvature_velocity;

    curvature_cal_val = a * state.v + b;


    if (curvature_cal_val < min_curvature_gain) {
        curvature_cal_val = min_curvature_gain;
    }

    target_curvature = std::atan(target_curvature * curvature_cal_val);

    target_curvature = target_curvature * lpf_tau + past_curvature * (1 - lpf_tau);

    // 타겟 조향 각도와 현재 조향각 에러 값을 통한 pid 계산
    float error = target_curvature - this->state.steer;
    output = this->state.steer + error * 0.65;

    output += this->distance_error * this->yaw_kp + (this->distance_error - past_distance_error) * this->yaw_kd;

    past_distance_error = this->distance_error;

    this->yaw_pre_e = error;
    past_curvature = target_curvature;

    return output;
}

float curvature_steer_control::velocity_control(pt_control_state_t state, path_point_t target_point)
{
    return target_point.speed;
}
