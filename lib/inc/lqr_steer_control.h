#ifndef LQR_STEER_CONTROL_H
#define LQR_STEER_CONTROL_H

#include <vector>
#include <fstream>

#include "cubic_spline_planner.h"
#include "lqr_pid_control.h"
#include "path_manager.h"


class lqr_steer_control
{
public:
    lqr_steer_control();
    ~lqr_steer_control();

public:
    void init(const double max_steer_angle, const double max_speed, const double wheel_base);
    bool update(double dt);
    void set_course(ControlState init_state, std::vector<Point> points);
    void add_course(ControlState init_state, std::vector<Point> points);
    double calculate_error();

private:
    int calculate_nearest_index(ControlState state, std::vector<Point> points, int pind, double& min_distance);
    void smooth_yaw(std::vector<Point> &points);
    ControlState update_state(ControlState state, double a, double delta, double dt);
    ModelMatrix dlqr(ModelMatrix A, ModelMatrix B, ModelMatrix Q, ModelMatrix R);
    int lqr_steering_control(ControlState state, double& steer, double& pe, double& pth_e);
    ModelMatrix solve_DARE(ModelMatrix A, ModelMatrix B, ModelMatrix Q, ModelMatrix R);
    ModelMatrix Q;
    ModelMatrix R;

private:
    double t;                  // 누적 시간
    double dt;

    ControlState init_state;
    ControlState goal_state;
    std::vector<Point> points; // spline 된 좌표값 + yaw + speed, 굴곡

    ControlState state;        // 현재 상태
    int target_ind;             // 목표로 가려는 point의 index값
    double dl;                  //
    std::vector<double> oa;     // accel
    std::vector<double> odelta; // steer
    pid_controller path_pid;

    double max_steer_angle;
    double max_speed;
    double wheel_base;
    double pid_gain;

public :
    ControlState get_state() const {
        return this->state;
    }

    void set_state(ControlState state) {
        this->state = state;
    }

    void set_target_index(int target_index) {
        this->target_ind = target_index;
    }

    void set_q(int i, int j, double value) {
        this->Q.set(i, j, value);
    }

    void set_r(int i, int j, double value) {
        this->R.set(i, j, value);
    }

    double get_q(int i, int j) {
        return this->Q.get(i, j).to_double();
    }

    double get_r(int i, int j) {
        return this->R.get(i, j).to_double();
    }

    std::vector<Point> get_points() const {
        return this->points;
    }

    int get_target_index() const {
        return this->target_ind;
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

    void get_distance_gain(double* kp, double* ki, double* kd) {
        *kp = this->get_q(0, 0);
        *ki = this->get_r(0, 0);
        *kd = this->get_q(2, 2);
    }
    void set_distance_gain(double kp, double ki, double kd) {
        this->set_q(0, 0, kp);
        this->set_q(2, 2, kd);
        this->set_r(0, 0, kd);
    }
    void get_steer_gain(double* kp, double* ki, double* kd) {
        *kp = this->get_q(0, 0);
        *ki = this->get_r(0, 0);
        *kd = this->get_q(2, 2);
    }
    void set_steer_gain(double kp, double ki, double kd) {
        this->set_q(0, 0, kp);
        this->set_q(2, 2, kd);
        this->set_r(0, 0, kd);
    }
};

#endif // LQR_STEER_CONTROL_H
