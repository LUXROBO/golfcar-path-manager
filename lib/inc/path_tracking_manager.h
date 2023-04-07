#ifndef PATH_TRACKING_MANAGER_H
#define PATH_TRACKING_MANAGER_H

#include <vector>
#include <fstream>

#include "cubic_spline_planner.h"
#include "pid_control.h"
#include "path_manager.h"

#define _USE_MATH_DEFINES

class path_tracking_manager
{
public:
    path_tracking_manager();
    ~path_tracking_manager();

public:
    virtual bool update(double dt);
    void add_course(ControlState init_state, std::vector<Point> points);

protected:
    virtual int calculate_nearest_index(ControlState state, std::vector<Point> points, int pind);
    void smooth_yaw(std::vector<Point> &points);
    ControlState update_state(ControlState state, double a, double delta, double dt);

protected:
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
