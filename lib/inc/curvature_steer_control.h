#pragma once

// std
#include <vector>
#include <fstream>


#include "lqr_pid_control.h"
#include "path_manager.h"

#define MAX_DRIVE_SPEED   (18.0f / 3.6f)
#define FAST_DRIVE_SPED   (12.0f / 3.6f)
#define HIGH_DRIVE_SPEED  (12.0f / 3.6f)
#define MID_DRIVE_SPEED   (7.5f  / 3.6f)
#define SLOW_DRIVE_SPEED  (4.5f  / 3.6f)
#define LOW_DRIVE_SPEED   (3.5f  / 3.6f)

#define MIN(x,y) (((x) < (y)) ? (x) : (y) )
#define MAX(x,y) (((x) > (y)) ? (x) : (y) )

template <typename T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
    return (v < lo) ? lo : (hi < v) ? hi : v;
}
class curvature_steer_control : public path_tracker
{
public:
    curvature_steer_control();
    curvature_steer_control(const float max_steer_angle, const float max_speed, const float wheel_base, const float center_to_gps_distance);
    ~curvature_steer_control();

public:
    virtual void set_gain(int gain_index, float* gain_value);
    virtual void get_gain(int gain_index, float* gain_value);

    path_point_t test_function(path_point_t current, path_point_t target_point);
private:
    virtual float steering_control(pt_control_state_t state, std::vector<path_point_t> target_point, uint8_t mode);
    virtual float constrained_steering_control(pt_control_state_t state, std::vector<path_point_t> target_point, uint8_t mode, float dt);
    virtual float velocity_control(pt_control_state_t state, path_point_t target_point);
private:
    pid_controller path_yaw_pid;
    pid_controller path_distance_pid;

    float yaw_kp;
    float yaw_ki;
    float yaw_kd;
    float yaw_pre_e;

public:
    path_point_t past_path_circle;
};
