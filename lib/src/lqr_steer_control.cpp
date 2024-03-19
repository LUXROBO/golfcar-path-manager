#include "lqr_steer_control.h"

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <algorithm>


lqr_steer_control::lqr_steer_control()
{
    this->Q = ModelMatrix::identity(3, 3);
    this->R = ModelMatrix::identity(1, 1);
}

lqr_steer_control::lqr_steer_control(const double max_steer_angle, const double max_speed, const double wheel_base, const double center_to_gps_distance = 0)
: path_tracker(max_steer_angle, max_speed, wheel_base, center_to_gps_distance)
{
    this->Q = ModelMatrix::identity(3, 3);
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

double lqr_steer_control::steering_control(pt_control_state_t state, path_point_t target_point)
{
    /* state matrix
    x = [position_x
         position_y
         yaw ]

    u = [angular velocity]

    A = [1,  0, -sin(yaw) * v * dt,
         0,  1,  cos(yaw) * v * dt,
         0,  0,                  1 ] (jacobian)
    */
    ModelMatrix A = ModelMatrix::zero(3, 3);
    A.set(0, 0, 1.0);
    A.set(0, 2, -sin(state.yaw) * target_point.speed * this->dt);
    A.set(1, 1, 1.0);
    A.set(1, 2, cos(state.yaw) * target_point.speed * this->dt);
    A.set(2, 2, 1);

    ModelMatrix B = ModelMatrix::zero(3, 1);
    B.set(2, 0, dt);
    ModelMatrix K = dlqr(A, B, this->Q, this->R);
    ModelMatrix x = ModelMatrix::zero(3, 1);

    x.set(0, 0, target_point.x - state.x);
    x.set(1, 0, target_point.y - state.y);
    x.set(2, 0, pi_to_pi(target_point.yaw - state.yaw));

    double target_angular_velocity = (K * x).get(0, 0).to_double();
    double steer = atan(target_angular_velocity * this->wheel_base / state.v);

    return steer;
}

double lqr_steer_control::velocity_control(pt_control_state_t state, path_point_t target_point)
{
    double accel = target_point.speed - state.v;

    return accel;
}

void lqr_steer_control::set_gain(int gain_index, double* gain_value)
{
    if (gain_index == PT_GAIN_TYPE_LQR_Q) {
        for (unsigned int i = 0; i < this->Q.row(); i++) {
            this->Q.set(i, i, gain_value[i]);
        }
    } else if (gain_index == PT_GAIN_TYPE_LQR_R)  {
        for (unsigned int i = 0; i < this->R.row(); i++) {
            this->R.set(i, i, gain_value[i]);
        }
    }
}

void lqr_steer_control::get_gain(int gain_index, double* gain_value)
{
    if (gain_index == PT_GAIN_TYPE_LQR_Q) {
        for (unsigned int i = 0; i < this->Q.row(); i++) {
            gain_value[i] = this->Q.get(i, i).to_double();
        }
    } else if (gain_index == PT_GAIN_TYPE_LQR_R)  {
        for (unsigned int i = 0; i < this->R.row(); i++) {
            gain_value[i] = this->R.get(i, i).to_double();
        }
    }
}
