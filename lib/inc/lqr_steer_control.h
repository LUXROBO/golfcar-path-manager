#ifndef LQR_STEER_CONTROL_H
#define LQR_STEER_CONTROL_H

#include <vector>
#include <fstream>

#include "cubic_spline_planner.h"
#include "lqr_pid_control.h"
#include "path_manager.h"

#define _USE_MATH_DEFINES

class lqr_steer_control
{
public:
    lqr_steer_control();
    ~lqr_steer_control();

public:
    bool update(float dt);
    void generate_spline(ControlState init_state, std::vector<WayPoint> waypoints, float target_speed, float ds=1.0);
    void add_course(ControlState init_state, std::vector<Point> points);
    void calc_ref_trajectory(float dt, ModelMatrix& reference_point, ModelMatrix& reference_steer);
    float calculate_error();

private:
    int calculate_nearest_index(ControlState state, std::vector<Point> points, int pind, float& min_distance);
    void smooth_yaw(std::vector<Point> &points);
    ModelMatrix predict_motion(ControlState x0, float a, float delta, ModelMatrix x_ref, float dt);
    void linear_mpc_control(ModelMatrix x_ref, ModelMatrix x_bar, ControlState x0, ModelMatrix d_ref);
    ControlState update_state(ControlState state, float a, float delta, float dt);
    ModelMatrix dlqr(ModelMatrix A, ModelMatrix B, ModelMatrix Q, ModelMatrix R);
    int lqr_steering_control(ControlState state, float& steer, float& pe, float& pth_e);
    ModelMatrix solve_DARE(ModelMatrix A, ModelMatrix B, ModelMatrix Q, ModelMatrix R);

private:
    float t;                  // 누적 시간
    float dt;
    ControlState init_state;
    ControlState goal_state;
    std::vector<Point> points; // spline 된 좌표값 + yaw + speed, 굴곡

    ControlState state;        // 현재 상태
    int target_ind;             // 목표로 가려는 point의 index값
    float dl;                  //
    std::vector<float> oa;     // accel
    std::vector<float> odelta; // steer
    pid_controller path_pid;

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

#endif // LQR_STEER_CONTROL_H
