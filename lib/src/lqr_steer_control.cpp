#include "lqr_steer_control.h"

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <algorithm>


static const double DEFAULT_MAX_STEER = 45.0 * M_PI / 180.0;     // [rad] 45deg
static const double DEFAULT_MAX_SPEED = 10.0 / 3.6;              // [ms] 10km/h
static const double DEFAULT_WHEEL_BASE = 0.41;                   // 앞 뒤 바퀴 사이 거리 [m]
static const double DEFAULT_PID_GAIN = 1;                      // gain

static double distance_between_point_and_line(ControlState point, Point line_point1, Point line_point2)
{
    double a = (line_point1.y - line_point2.y) / (line_point1.x - line_point2.x);
    double c = line_point1.y - a * line_point1.x;
    double b = -1;

    return abs(a * point.x + b * point.y + c) / sqrt(a * a + b * b);
}


lqr_steer_control::lqr_steer_control()
{
    this->path_pid = pid_controller(1, 0, 0);
    this->Q = ModelMatrix::identity(4, 4);
    this->Q.set(2,2,5);
    this->R = ModelMatrix::identity(1, 1);
}

lqr_steer_control::~lqr_steer_control()
{

}

void lqr_steer_control::init(const double max_steer_angle, const double max_speed, const double wheel_base)
{
    this->path_pid = pid_controller(1, 0, 0);
    this->points.clear();

    this->max_steer_angle = max_steer_angle;
    this->max_speed = max_speed;
    this->wheel_base = wheel_base;
}

void lqr_steer_control::set_course(ControlState init_state, std::vector<Point> points)
{
	double xd = 0;
    int goal_index = points.size() - 1;

    this->init_state = init_state;
    this->points = points;

    if (this->init_state.yaw - this->points[0].yaw >= M_PI) {
        this->init_state.yaw -= 2.0f * M_PI;
    } else if (this->init_state.yaw - this->points[0].yaw <= -M_PI) {
        this->init_state.yaw += 2.0f * M_PI;
    }

    this->goal_state = ControlState(this->points[goal_index].x, this->points[goal_index].y, this->points[goal_index].yaw, 0, this->points[goal_index].speed);
    this->t = 0.0f;

    this->target_ind = this->calculate_nearest_index(this->init_state, this->points, 0, xd);
    this->smooth_yaw(this->points);
    this->oa.clear();
    this->odelta.clear();
}

void lqr_steer_control::add_course(ControlState init_state, std::vector<Point> points)
{
    double xd =0;
    this->points.insert(end(this->points), begin(points), end(points));

    int goal_index = points.size() - 1;

    this->init_state = init_state;  // init start state
    if (this->init_state.yaw - this->points[0].yaw >= M_PI) {
        this->init_state.yaw -= 2.0 * M_PI;
    } else if (this->init_state.yaw - this->points[0].yaw <= -M_PI) {
        this->init_state.yaw += 2.0 * M_PI;
    }
    this->goal_state = ControlState(this->points[goal_index].x, this->points[goal_index].y, this->points[goal_index].yaw, 0, this->points[goal_index].speed);
    this->t = 0.0;

    this->target_ind = this->calculate_nearest_index(this->init_state, this->points, 0, xd);
    this->smooth_yaw(this->points);
    this->oa.clear();
    this->odelta.clear();
    this->points.back().speed = 2.2;
}

int lqr_steer_control::calculate_nearest_index(ControlState state, std::vector<Point> points, int pind, double& min_distance_ref)
{
    const int N_IND_SEARCH = 10;
    double min = 10000;
    uint32_t min_point_index = 0;
    for (uint32_t i = pind; i < (pind + N_IND_SEARCH); i++) {
        double dx = state.x - points[i].x;
        double dy = state.y - points[i].y;
        double distance_2 = dx * dx + dy * dy;
        if (min > distance_2) {
            min = distance_2;
            min_point_index = i;
        }
        if (i >= (points.size() - 1)) {
            break;
        }
    }

    double min_distance = min;
    uint32_t min_index = min_point_index;

    double dxl = points[min_index].x - state.x;
    double dyl = points[min_index].y - state.y;

    double angle = 0;

    if (min_index != 0) {
        min_distance = distance_between_point_and_line(this->state, points[min_index], points[min_index - 1]);
    }

    if ((abs(dxl) <= 0.00001) && (abs(dyl) <= 0.00001)) {
        angle = pi_2_pi(points[min_index].yaw);
    } else {
        angle = pi_2_pi(points[min_index].yaw - atan2(dyl, dxl));
    }

    if (angle < 0) {
        min_distance *= -1;
    }
    min_distance_ref = min_distance;

    return min_index;
}

void lqr_steer_control::smooth_yaw(std::vector<Point> &points)
{
    for (uint32_t i = 0; i < points.size() - 1; i++)
    {
        double diff_yaw = points[i + 1].yaw - points[i].yaw;

        while (diff_yaw >= M_PI_2) {
            points[i + 1].yaw -= M_PI * 2.0;
            diff_yaw = points[i + 1].yaw - points[i].yaw;
        }

        while (diff_yaw <= -M_PI_2) {
            points[i + 1].yaw += M_PI * 2.0;
            diff_yaw = points[i + 1].yaw - points[i].yaw;
        }
    }
}

ModelMatrix lqr_steer_control::solve_DARE(ModelMatrix A, ModelMatrix B, ModelMatrix Q, ModelMatrix R)
{
    ModelMatrix X = Q;
    int maxiter = 10;
    q_format eps = 0.01;

    for (int i = 0; i < maxiter; i++) {
        // Xn =          A.T @ X @ A           - A.T @ X @ B @ la.inv(R + B.T @ X @ B) @ B.T @ X @ A + Q
        ModelMatrix Xn = (A.transpose() * X * A) - A.transpose() * X * B * (R + (B.transpose() * X * B)).inverse() * B.transpose() * X * A + Q;
        ModelMatrix riccati_equ = Xn - X;
        q_format max = 0;
        for (uint32_t j = 0; j < riccati_equ.row(); j++) {
            for (uint32_t k = 0; k < riccati_equ.column(); k++) {
                q_format element = riccati_equ.get(j, k).abs();
                if (max < element) {
                    max = element;
                }
            }
        }
        if (max < eps) {
            return Xn;
        }
        X = Xn;
    }
    return X;
}

ModelMatrix  lqr_steer_control::dlqr(ModelMatrix A, ModelMatrix B, ModelMatrix Q, ModelMatrix R)
{
    ModelMatrix X = solve_DARE(A, B, Q, R);
    //              la.inv(B.T @ X @ B + R)               @ (B.T @ X @ A)
    ModelMatrix K = (((B.transpose() * X * B) + R).inverse()) * (B.transpose() * X * A);
    return K;
}

int lqr_steer_control::lqr_steering_control(ControlState state, double& steer, double& pe, double& pth_e)
{
    double e = 0;
    this->target_ind = this->calculate_nearest_index(state, this->points, this->target_ind, e);
    int jump_point = this->target_ind + 0;
    if (jump_point > (this->points.size() - 1)) {
        jump_point = this->target_ind;
    }

    q_format k = this->points[jump_point].k;
    q_format v = state.v;
    double tttt = pi_2_pi(state.yaw - this->points[jump_point].yaw);
    q_format th_e = tttt;

    q_format L = this->wheel_base;

    ModelMatrix A = ModelMatrix::zero(4, 4);
    A.set(0, 0, 1.0);
    A.set(0, 1, this->dt);
    A.set(1, 2, v);
    A.set(2, 2, 1.0);
    A.set(2, 3, this->dt);

    ModelMatrix B = ModelMatrix::zero(4, 1);
    B.set(3, 0, v / L);

    ModelMatrix K = dlqr(A, B, this->Q, this->R);

    ModelMatrix x = ModelMatrix::zero(4, 1);

    x.set(0, 0, e);
    x.set(1, 0, (e - pe) / this->dt);
    x.set(2, 0, th_e);
    x.set(3, 0, (th_e - pth_e) / this->dt);

    double ff = atan2((L * k).to_double(), 1);
    double fb = pi_2_pi((-1 * K * x).get(0, 0).to_double());

    steer = ff + fb;

    pe = e;
    th_e = th_e;

    return jump_point;
}

bool lqr_steer_control::update(double dt) {
    // dt 저장
    ModelMatrix reference_point;
    ModelMatrix reference_steer;

    double calculated_steer = 0;
    static double pe = 0;
    static double pth_e = 0;
    this->dt = dt;
    if (dt == 0) {
        return false;
    }

    uint32_t closest_point_index = lqr_steering_control (this->state, calculated_steer, pe, pth_e);
    // PID로 가속도 값 계산
    this->path_pid.set_target(this->points[closest_point_index].speed);

    double calculated_accel = this->path_pid.calculate(this->state.v);
    // state update
    this->state = this->update_state(this->state, calculated_accel, calculated_steer, this->dt);
    double state_to_goal_distance = sqrt(pow(this->goal_state.x - this->state.x, 2) + pow(this->goal_state.y - this->state.y, 2));
    size_t remain_point = get_remain_point();

    if (remain_point == 0) {
        // finish
        return true;
    }
    return false;
}

double lqr_steer_control::calculate_error()
{
    double e = 0;
    double distance1 = 100;
    double distance2 = 100;
    int temp_target_index = this->target_ind - 10;
    if (temp_target_index < 0) {
        temp_target_index = 0;
    }
    int nearest_index = this->calculate_nearest_index(state, this->points, temp_target_index, e);

    auto cal_diatance_between_line_to_point = [](double x1, double y1, double x2, double y2, double stand_x, double stand_y ) -> double{
        double a = (y1 - y2) / (x1 - x2);
        double b = -1;
        double c = y1 - a * x1;

        return abs(a * stand_x + b * stand_y + c) / sqrt(a*a + 1);
    };

    if (nearest_index != 0) {
        distance1 = cal_diatance_between_line_to_point(this->points[nearest_index].x, this->points[nearest_index].y,
                                           this->points[nearest_index - 1].x, this->points[nearest_index - 1].y,
                                           this->state.x, this->state.y);
    }
    if (nearest_index != (this->points.size() - 1)) {
        distance2 = cal_diatance_between_line_to_point(this->points[nearest_index].x, this->points[nearest_index].y,
                                           this->points[nearest_index + 1].x, this->points[nearest_index + 1].y,
                                           this->state.x, this->state.y);
    }
    return distance1 < distance2 ? distance1 : distance2;
}

ControlState lqr_steer_control::update_state(ControlState state, double accel, double steer_delta, double dt)
{
    if (steer_delta > this->max_steer_angle) {
        steer_delta = this->max_steer_angle;
    } else if (steer_delta < -this->max_steer_angle) {
        steer_delta = -this->max_steer_angle;
    }
    state.steer = steer_delta;
    // golfcar position, angle update
    state.x = state.x + state.v * std::cos(state.yaw) * dt;
    state.y = state.y + state.v * std::sin(state.yaw) * dt;
    state.yaw = state.yaw + state.v / this->wheel_base * std::tan(steer_delta) * dt;
    state.v = state.v + accel * dt;

    if (state.v > this->max_speed) {
        state.v = this->max_speed;
    } else if (state.v < -this->max_speed) {
        state.v = -this->max_speed;
    }

    return state;
}
