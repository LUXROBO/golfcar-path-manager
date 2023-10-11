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
    void set_course(ControlState init_state, std::vector<Point> points);
    void add_course(ControlState init_state, std::vector<Point> points);
    double calculate_error();

private:
    int calculate_nearest_index(ControlState state, std::vector<Point> points, int pind);
    int calculate_target_index(ControlState state, std::vector<Point> points, int pind, double& err_front_axel);
    void smooth_yaw(std::vector<Point> &points);
    ControlState update_state(ControlState state, double a, double delta, double dt);
    int pid_steering_control(ControlState state, int target_index, double distance_error, double& steer);

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
    pid_controller path_distance_pid;

    double max_steer_angle;
    double max_speed;
    double wheel_base;

    double steer_kp;
    double steer_ki;
    double steer_kd;
    double steer_pre_e;

    int jumping_point_count;
    double ref_distance;

    double adapted_pid_distance_gain;
    double adapted_pid_steer_gain;

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

    void get_distance_gain(double* kp, double* ki, double* kd) {
        *kp = this->path_distance_pid.get_p_gain();
        *ki = this->path_distance_pid.get_i_gain();
        *kd = this->path_distance_pid.get_d_gain();
    }

    void set_distance_gain(double kp, double ki, double kd) {
        this->path_distance_pid.set_gain(kp, ki, kd);
    }

    void get_steer_gain(double* kp, double* ki, double* kd) {
        *kp = this->steer_kp;
        *ki = this->steer_ki;
        *kd = this->steer_kd;
    }

    void set_steer_gain(double kp, double ki, double kd) {
        this->steer_kp = kp;
        this->steer_ki = ki;
        this->steer_kd = kd;
    }
};

#endif // PID_STEER_CONTRL_H
