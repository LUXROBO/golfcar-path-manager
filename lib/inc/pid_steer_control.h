#ifndef PID_STEER_CONTRL_H
#define PID_STEER_CONTRL_H

#include <vector>
#include <fstream>

#include "cubic_spline_planner.h"
#include "lqr_pid_control.h"
#include "path_manager.h"

class pid_steer_control : public path_tracking_controller
{
public:
    enum pid_gain_select {
        distance,
        yaw,
        accel
    };
    pid_steer_control();
    ~pid_steer_control();

    int test_funtion();

private:
    virtual double steering_control(ControlState state, Point target_point);
    virtual double velocity_control(ControlState state, Point target_point);

private:
    pid_controller path_accel_pid;
    pid_controller path_distance_pid;

    double steer_kp;
    double steer_ki;
    double steer_kd;
    double steer_pre_e;

    double adapted_pid_distance_threshold;
    double adapted_pid_distance_gain;
    double adapted_pid_yaw_gain;

public:
    virtual void set_gain(int gain_index, double* gain_value);
    virtual void get_gain(int gain_index, double* gain_value);
};

#endif // PID_STEER_CONTRL_H
