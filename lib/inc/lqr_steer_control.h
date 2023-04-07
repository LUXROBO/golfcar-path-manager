#ifndef LQR_STEER_CONTROL_H
#define LQR_STEER_CONTROL_H

#include <vector>
#include <fstream>

#include "cubic_spline_planner.h"
#include "pid_control.h"
#include "path_manager.h"

#define _USE_MATH_DEFINES

class lqr_steer_control
{
public:
    lqr_steer_control();
    ~lqr_steer_control();

public:
    bool update(double dt);
    void generate_spline(ControlState init_state, std::vector<WayPoint> waypoints, double target_speed, double ds=1.0);
    void add_course(ControlState init_state, std::vector<Point> points);
    void calc_ref_trajectory(double dt, ModelMatrix& reference_point, ModelMatrix& reference_steer);
    double calculate_error();

private:
    int calculate_nearest_index(ControlState state, std::vector<Point> points, int pind);
    double lqr_steer_control::calculate_min_distance(int min_index);
    void smooth_yaw(std::vector<Point> &points);
    ControlState update_state(ControlState state, double a, double delta, double dt);
    ModelMatrix dlqr(ModelMatrix A, ModelMatrix B, ModelMatrix Q, ModelMatrix R);
    int lqr_steering_control(ControlState state, double& steer, double& pe, double& pth_e);
    ModelMatrix solve_DARE(ModelMatrix A, ModelMatrix B, ModelMatrix Q, ModelMatrix R);

private:
    double t;                  // 누적 시간
    double dt;
    ControlState init_state;
    ControlState goal_state;
    std::vector<Point> points; // spline 된 좌표값 + yaw + speed, 굴곡

    ControlState state;        // 현재 상태
    int target_ind;             // 목표로 가려는 point의 index값
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
