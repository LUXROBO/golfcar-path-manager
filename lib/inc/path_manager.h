#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H

#include <vector>
#include "model_matrix.h"
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
    double kp;
    double ki;
    double kd;
} pid_gain_t;

// typedef enum pid_type_{
//     pid_type_distance,
//     pid_type_steer,
// } pid_type;

#define PATH_TRACKER_PID_TYPE_DISTANCE 0
#define PATH_TRACKER_PID_TYPE_STEER 1

// typedef enum lqr_type_{
//     lqr_type_q,
//     lqr_type_r,
// } lqr_type;

#define PATH_TRACKER_LQR_TYPE_Q 0
#define PATH_TRACKER_LQR_TYPE_R 1

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
    bool get_target_steer(Point point, double* steer);

protected:
    int calculate_target_index(ControlState state, std::vector<Point> points, int pind);
    void smooth_yaw(std::vector<Point> &points);
    ControlState update_state(ControlState state, double a, double delta, double dt);
    bool is_point_in_correct_range(double dx, double dy, double yaw, double steer, double range_angle);
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

    double distance_error;
    double yaw_error;
    double steer_error;

    double target_steer;
    double target_velocity;

    double jumping_point;

public:
    ControlState get_state() const {
        return this->state;
    }

    void set_state(ControlState state) {
        this->state = state;
        // std::cout << "state -> x : " << this->state.x <<
        //                      " y : " << this->state.y <<
        //                      " yaw : " << this->state.yaw <<
        //                      " v : " << this->state.v <<
        //                      " steer : " << this->state.steer << std::endl;
    }

    void set_steer(double steer) {
        this->state.steer = steer;
    }

    void set_yaw(double yaw) {
        this->state.yaw = yaw;
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

    int get_jumped_target_index() const {
        if (this->target_ind + this->jumping_point >= this->points.size()) {
            return this->points.size();
        } else {
            return this->target_ind + this->jumping_point;
        }
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

    double get_distance_error() {
        return this->distance_error;
    }

    double get_target_steer() {
        return this->target_steer;
    }

    double get_target_velocity() {
        return this->target_velocity;
    }

    virtual void get_gain(int gain_index, double* gain_value);
    virtual void set_gain(int gain_index, double* gain_value);
};

#endif
