#ifndef STANLEY_STEER_CONTROL_H
#define STANLEY_STEER_CONTROL_H

#include <vector>
#include <fstream>

#include "cubic_spline_planner.h"
#include "pid_control.h"
#include "path_manager.h"

#define _USE_MATH_DEFINES

class pid_steer_control
{
public:
    pid_steer_control();
    ~pid_steer_control();

public:
    bool update(double dt);
    void generate_spline(ControlState init_state, std::vector<WayPoint> waypoints, double target_speed, double ds=1.0);
    void add_course(ControlState init_state, std::vector<Point> points);
    void calc_ref_trajectory(double dt, ModelMatrix& reference_point, ModelMatrix& reference_steer);
    double calculate_error();

private:
    int calculate_nearest_index(ControlState state, std::vector<Point> points, int pind);
    int calculate_target_index(ControlState state, std::vector<Point> points, int pind, double& err_front_axel);
    void smooth_yaw(std::vector<Point> &points);
    ControlState update_state(ControlState state, double a, double delta, double dt);
    int pid_steering_control(ControlState state, double& steer);

private:
    double t;                  // 누적 시간
    double dt;
    ControlState init_state;
    ControlState goal_state;
    std::vector<Point> points; // spline 된 좌표값 + yaw + speed, 굴곡
    int target_ind;             // 목표로 가려는 point의 index값
    ControlState state;        // 현재 상태
   
    double dl;                  //
    std::vector<double> oa;     // accel
    std::vector<double> odelta; // steer
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
