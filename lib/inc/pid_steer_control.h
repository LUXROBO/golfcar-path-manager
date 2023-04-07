#ifndef STANLEY_STEER_CONTROL_H
#define STANLEY_STEER_CONTROL_H

#include <vector>
#include <fstream>

#include "path_tracking_manager.h"
#include "cubic_spline_planner.h"
#include "pid_control.h"
#include "path_manager.h"

#define _USE_MATH_DEFINES

class pid_steer_control : public path_tracking_manager
{
public:
    pid_steer_control();
    ~pid_steer_control();

public:
    bool update(double dt);
    double calculate_error();

private:
    int calculate_nearest_index(ControlState state, std::vector<Point> points, int pind);
    int calculate_target_index(ControlState state, std::vector<Point> points, int pind, double& err_front_axel);
    int pid_steering_control(ControlState state, double& steer);
    pid_controller path_accel_pid;
    pid_controller path_steer_pid;
};

#endif // STANLEY_STEER_CONTROL_H
