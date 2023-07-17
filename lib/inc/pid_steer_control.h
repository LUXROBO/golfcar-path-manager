#ifndef PID_STEER_CONTRL_H
#define PID_STEER_CONTRL_H

#include <vector>
#include <fstream>

#include "cubic_spline_planner.h"
#include "lqr_pid_control.h"
#include "path_manager.h"

typedef struct pid_gain_{
    int pid_select;
    double kp;
    double ki;
    double kd;
} pid_gain_t;

// typedef struct pid_tracker_gain_{
//     int pid_select;
//     pid_gain_t gain;
// } pid_tracker_gain_t;

class pid_steer_control : public path_tracking_controller
{
public:
    enum pid_gain_select {
        distance,
        steer,
        accel
    };
    pid_steer_control();
    ~pid_steer_control();

    int test_funtion();

private:
    virtual int steering_control(ControlState state, double& steer);
    virtual int velocity_control(ControlState state, double& accel);

private:
    pid_controller path_accel_pid;
    pid_controller path_distance_pid;

    double steer_kp;
    double steer_ki;
    double steer_kd;
    double steer_pre_e;

public:
    virtual int set_gain(controller_gain_t gain);
    virtual int get_gain(controller_gain_t* gain);
};

#endif // PID_STEER_CONTRL_H
