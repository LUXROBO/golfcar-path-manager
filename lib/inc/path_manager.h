#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H

#include <vector>
// #include "resource.h"

#define M_PI    3.14159265358979323846
#define M_PI_2  1.57079632679489661923

typedef struct WayPoint_
{
    double x;
    double y;

    WayPoint_(double x, double y)
    {
        this->x = x;
        this->y = y;
    }

} WayPoint;

typedef struct Point_
{
    double x;
    double y;
    double yaw;
    double k;
    double speed;
} Point;


typedef struct ControlState_
{
    double x;
    double y;
    double yaw;
    double steer;
    double v;

    ControlState_()
    {

    }

    ControlState_(double x, double y, double yaw, double steer, double v)
    {
        this->x = x;
        this->y = y;
        this->yaw = yaw;
        this->steer = steer;
        this->v = v;
    }
} ControlState;

typedef struct pid_gain_{
    int pid_select;
    double kp;
    double ki;
    double kd;
} pid_gain_t;

class path_tracking_controller
{
public:
    path_tracking_controller();
    path_tracking_controller(const double max_steer_angle, const double max_speed, const double wheel_base);
    ~path_tracking_controller();
    void init(const double max_steer_angle, const double max_speed, const double wheel_base);
    bool update(double dt);
    void set_course(ControlState init_state, std::vector<Point> points);
    void add_course(ControlState init_state, std::vector<Point> points);
    double pi_2_pi(double angle);

protected:
    int calculate_target_index(ControlState state, std::vector<Point> points, int pind, double& err_front_axel);
    void smooth_yaw(std::vector<Point> &points);
    ControlState update_state(ControlState state, double a, double delta, double dt);
    virtual int steering_control(ControlState state, double& steer);
    virtual int velocity_control(ControlState state, double& accel);

protected:
    double dt;
    ControlState init_state;
    ControlState goal_state;
    std::vector<Point> points;
    int target_ind;
    ControlState state;

    double max_steer_angle;
    double max_speed;
    double wheel_base;

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

    virtual void get_gain(void*);
    virtual void set_gain(void*);
};

#endif
