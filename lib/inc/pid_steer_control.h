#ifndef STANLEY_STEER_CONTROL_H
#define STANLEY_STEER_CONTROL_H

#include <vector>
#include <fstream>

#include "cubic_spline_planner.h"
#include "lqr_pid_control.h"
#include "path_manager.h"


#define _USE_MATH_DEFINES

class pid_steer_control
{
public:
    pid_steer_control();
    ~pid_steer_control();

public:
    bool update(float dt);
    void generate_spline(ControlState init_state, std::vector<WayPoint> waypoints, float target_speed, float ds=1.0);
    void add_course(ControlState init_state, std::vector<Point> points);
    void calc_ref_trajectory(float dt, ModelMatrix& reference_point, ModelMatrix& reference_steer);
    float calculate_error();

private:
    int calculate_nearest_index(ControlState state, std::vector<Point> points, int pind);
    int calculate_target_index(ControlState state, std::vector<Point> points, int pind, float& err_front_axel);
    void smooth_yaw(std::vector<Point> &points);
    ControlState update_state(ControlState state, float a, float delta, float dt);
    int pid_steering_control(ControlState state, float& steer);

private:
    float t;                  // 누적 시간
    float dt;
    ControlState init_state;
    ControlState goal_state;
    std::vector<Point> points; // spline 된 좌표값 + yaw + speed, 굴곡
    int target_ind;             // 목표로 가려는 point의 index값
    ControlState state;        // 현재 상태

    float dl;                  //
    std::vector<float> oa;     // accel
    std::vector<float> odelta; // steer
    pid_controller path_accel_pid;
    pid_controller path_steer_pid;

public :
    ControlState get_state() const {
        return this->state;
    }

    void set_state(ControlState state) {
        this->state = state;
    }

    std::vector<Point> get_points() const {
        return this->points;
    }
};

#endif // STANLEY_STEER_CONTROL_H
