#include "lqr_steer_control.h"

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <algorithm>


static const double DEFAULT_MAX_STEER = 45.0 * M_PI / 180.0;     // [rad] 45deg
static const double DEFAULT_MAX_SPEED = 10.0 / 3.6;              // [ms] 10km/h
static const double DEFAULT_WHEEL_BASE = 0.41;                   // 앞 뒤 바퀴 사이 거리 [m]
static const double DEFAULT_PID_GAIN = 1;                      // gain

lqr_steer_control::lqr_steer_control()
{
    this->Q = ModelMatrix::identity(4, 4);
    this->R = ModelMatrix::identity(1, 1);
}

lqr_steer_control::~lqr_steer_control()
{

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

int lqr_steer_control::steering_control(ControlState state, double& steer)
{
    double e = 0;
    static double pe = 0;
    static double pth_e = 0;
    this->target_ind = this->calculate_target_index(state, this->points, this->target_ind, e);
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

int lqr_steer_control::velocity_control(ControlState state, double& accel)
{
    accel = this->points[this->target_ind].speed - state.v;

    return 0;
}

void lqr_steer_control::set_gain(void* gain)
{
    lqr_gain_t received_gain = *(lqr_gain_t*)gain;
    
    if (received_gain.lqr_select <= (int)lqr_steer_control::lqr_gain_select::q) {
        this->Q = received_gain.weighting_matrix;
    } else if (received_gain.lqr_select == lqr_steer_control::lqr_gain_select::r) {
        this->R = received_gain.weighting_matrix;
    }
}

void lqr_steer_control::get_gain(void* gain)
{
    lqr_gain_t* received_gain = (lqr_gain_t*)gain;
    if (received_gain->lqr_select <= (int)lqr_steer_control::lqr_gain_select::q) {
        received_gain->weighting_matrix = this->Q;
    } else if (received_gain->lqr_select == lqr_steer_control::lqr_gain_select::r) {
        received_gain->weighting_matrix = this->R;
    }
}
