#include "yaw_filter.h"
#include "model_matrix.h"
#include <cmath>
#include <iostream>

typedef struct yaw_filter_context_
{
    ModelMatrix x;
    // ModelMatrix predict_x;
    // ModelMatrix estimate_x;
    ModelMatrix Q;
    ModelMatrix R;
    ModelMatrix H;
    ModelMatrix P;
    ModelMatrix K;
    ModelMatrix S_inv;
} yaw_filter_context_t;

yaw_filter_context_t yaw_estimate_filter;
int state_member = 2; // [yaw, angular velocity]

bool yaw_filter_init()
{
    yaw_estimate_filter.x = ModelMatrix::zero(state_member, 1);
    yaw_estimate_filter.P = ModelMatrix::identity(state_member, state_member);
    yaw_estimate_filter.Q = ModelMatrix::zero(state_member, state_member);
    yaw_estimate_filter.R = ModelMatrix::zero(state_member, state_member);

    yaw_estimate_filter.K = ModelMatrix::zero(state_member, state_member);
    yaw_estimate_filter.S_inv = ModelMatrix::zero(1, 1);

    float H_array[] = {1, 0};
    yaw_estimate_filter.H = ModelMatrix(1, 2, H_array);
    return true;
}

void yaw_filter_set_yaw(float yaw)
{

    yaw_estimate_filter.x.set(0, 0, yaw);
}

float yaw_filter_predict(float angular_velocity, float dt)
{
    float A_array[4] = {1, dt,
                        0, 0};
    ModelMatrix A = ModelMatrix(state_member, state_member, A_array);
    yaw_estimate_filter.x = A * yaw_estimate_filter.x;
    yaw_estimate_filter.x.set(0, 0, path_tracker::pi_to_pi(yaw_estimate_filter.x.get(0, 0)));
    yaw_estimate_filter.x.set(1, 0, angular_velocity); // 각속도 업데이트
    yaw_estimate_filter.P = A * yaw_estimate_filter.P * A.transpose() + yaw_estimate_filter.Q;

    return yaw_estimate_filter.x.get(0, 0); // yaw를 리턴
}

bool position_filter_valid_gate(ModelMatrix innovation, ModelMatrix H, float sigma)
{
    yaw_estimate_filter.S_inv = (H * yaw_estimate_filter.P * H.transpose() + yaw_estimate_filter.R).inverse();
    ModelMatrix temp = yaw_estimate_filter.S_inv;

    float V = (innovation.transpose() * yaw_estimate_filter.S_inv * innovation).get(0, 0);
    return V <= (sigma * sigma);
}

float yaw_filter_estimate(float yaw, float sigma)
{
    ModelMatrix z = ModelMatrix::zero(5, 1);
    if (abs(yaw_estimate_filter.x.get(0, 0) - yaw) > PT_M_PI){
        if (yaw < 0){
            yaw += PT_M_PI *2;
        } else{
            yaw -= PT_M_PI *2;
        }
    }
    z.set(0, 0, yaw);
    ModelMatrix innovation = z - yaw_estimate_filter.H * yaw_estimate_filter.x;
    if (position_filter_valid_gate(innovation, yaw_estimate_filter.H, sigma)) {
        return estimate(z);
    } else {
        return yaw_estimate_filter.x.get(0, 0);
    }
}

float estimate(ModelMatrix z)
{
    ModelMatrix C_hat = ModelMatrix::zero(state_member, state_member);
    ModelMatrix C_x_hat = ModelMatrix::zero(state_member, state_member);
    ModelMatrix S = ModelMatrix::zero(state_member, state_member);
    yaw_estimate_filter.K = yaw_estimate_filter.P * yaw_estimate_filter.H.transpose() * yaw_estimate_filter.S_inv;

    yaw_estimate_filter.x = yaw_estimate_filter.x +
                                            yaw_estimate_filter.K * (z - yaw_estimate_filter.H * yaw_estimate_filter.x);
    yaw_estimate_filter.x.set(0, 0, path_tracker::pi_to_pi(yaw_estimate_filter.x.get(0, 0)));
    yaw_estimate_filter.P = yaw_estimate_filter.P - yaw_estimate_filter.K * yaw_estimate_filter.H * yaw_estimate_filter.P;
    return yaw_estimate_filter.x.get(0, 0);
}
