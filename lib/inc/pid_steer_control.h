#ifndef PID_STEER_CONTRL_H
#define PID_STEER_CONTRL_H

#include <vector>
#include <fstream>

#include "cubic_spline_planner.h"
#include "lqr_pid_control.h"
#include "path_manager.h"


class pid_steer_control
{
public:
    pid_steer_control();
    ~pid_steer_control();

public:
    void init(const double max_steer_angle, const double max_speed, const double wheel_base);
    bool update(double dt);
    void generate_spline(ControlState init_state, std::vector<WayPoint> waypoints, double target_speed, double ds=1.0);
    void set_course(ControlState init_state, std::vector<Point> points);
    void add_course(ControlState init_state, std::vector<Point> points);
    double calculate_error();

private:
    int calculate_nearest_index(ControlState state, std::vector<Point> points, int pind);
    int calculate_target_index(ControlState state, std::vector<Point> points, int pind, double& err_front_axel);
    void smooth_yaw(std::vector<Point> &points);
    ControlState update_state(ControlState state, double a, double delta, double dt);
    int pid_steering_control(ControlState state, double& steer);

private:
    double t;                    // 누적 시간
    double dt;
    ControlState init_state;
    ControlState goal_state;
    std::vector<Point> points;  // spline 된 좌표값 + yaw + speed, 굴곡
    int target_ind;             // 목표로 가려는 point의 index값
    ControlState state;         // 현재 상태

    double dl;
    std::vector<double> oa;     // accel
    std::vector<double> odelta; // steer
    pid_controller path_accel_pid;
    pid_controller path_steer_pid;

    double max_steer_angle;
    double max_speed;
    double wheel_base;

    double kp;
    double ki;
    double kd;
    double pre_e;

public:
    ControlState get_state() const {
        return this->state;
    }

    void set_state(ControlState state) {
        this->state = state;
    }

    void set_target_index(int target_index) {
        this->target_ind = target_index;
    }

    std::vector<Point> get_points() const {
        return this->points;
    }

    void remove_points(size_t num) {
        if (num == 0) {
            this->points.clear();
        } else if (this->points.size() > num) {
            this->points.erase(begin(this->points), begin(this->points) + num);
        }
    }

    size_t get_remain_point() const {
        return this->points.size() - target_ind - 1;
    }

    void get_gain(double* kp, double* ki, double* kd) {
        *kp = this->kp;
        *ki = this->ki;
        *kd = this->kd;
    }

    void set_gain(double kp, double ki, double kd) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }
};

#endif // PID_STEER_CONTRL_H
