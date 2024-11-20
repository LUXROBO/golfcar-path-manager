#include "velocity_filter.h"
#include "model_matrix.h"
#include <cmath>
#include <iostream>

typedef struct velocity_filter_context_
{
    // float A;       /** 상태 전이 함수 */
    ModelMatrix H;          /** 측정 상태 공간 방정식 */
    ModelMatrix P;        /** 측정 에러 예측 공분산 */
    ModelMatrix K;          /** 칼만 게인 */
    ModelMatrix x;          /** state */

    ModelMatrix Q;          /** 예측 노이즈 */
    ModelMatrix R;          /** 측정 노이즈 */
    ModelMatrix S_inv;      /** 칼만 게인 계산용 */
    ModelMatrix z;

    float velocity;
    float last_update_time;
    float chi_square_value;
    int init_flag;
} velocity_filter_context_t;

velocity_filter_context_t velocity_estimate_filter;

/*
kalman filter innovation -> 측정 값과 실제 값 차이
kalman filter residual ->   예측 값과 측정 값 차이
*/

/**
 * @brief
 *
 * @param z
 * @return pt_control_state_t
 */
static float estimate(ModelMatrix z);

bool velocity_filter_init()
{
    velocity_estimate_filter.P = ModelMatrix::identity(2, 2);

    velocity_estimate_filter.x = ModelMatrix::zero(2, 1);
    velocity_estimate_filter.R = ModelMatrix::zero(2, 2);//0.07;
    velocity_estimate_filter.Q = ModelMatrix::zero(2, 2);//0.005;
    velocity_estimate_filter.K = ModelMatrix::zero(2, 2);//0;
    velocity_estimate_filter.H = ModelMatrix::identity(2, 2);//1;
    velocity_estimate_filter.z = ModelMatrix::zero(2, 1);

    velocity_estimate_filter.S_inv = ModelMatrix::identity(2, 2);//;

    float Q_array[4] = {0.005, 0,
                         0   , 0.005};
    velocity_estimate_filter.Q = ModelMatrix(2, 2, Q_array);

    float R_array[4] = {0.03, 0,
                         0   , 0.03};
    velocity_estimate_filter.R = ModelMatrix(2, 2, R_array);

    return true;
}

void velocity_filter_set_last_update_time(float update_time)
{
    velocity_estimate_filter.last_update_time = update_time;
}

float velocity_filter_get_velocity()
{
    return velocity_estimate_filter.velocity;
}

float velocity_filter_get_chi_square_value()
{
    return velocity_estimate_filter.chi_square_value;
}

void velocity_filter_set_velocity(float v_x, float v_y)
{
    velocity_estimate_filter.init_flag = 1;
    velocity_estimate_filter.x.set(0, 0, v_x);
    velocity_estimate_filter.x.set(1, 0, v_y);
    velocity_estimate_filter.velocity = std::sqrt(pow(v_x, 2) + pow(v_y, 2));
    velocity_estimate_filter.P = ModelMatrix::zero(2, 2);
}

float velocity_filter_predict_state(float accel_x, float accel_y, float updated_time)
{
    if (velocity_estimate_filter.init_flag == 1){
        float dt = updated_time - velocity_estimate_filter.last_update_time;
        velocity_estimate_filter.last_update_time = updated_time;

        float A = 1;
        float v_x = velocity_estimate_filter.x.get(0, 0) + accel_x * dt;
        float v_y = velocity_estimate_filter.x.get(1, 0) + accel_y * dt;
        velocity_estimate_filter.x.set(0, 0, v_x);
        velocity_estimate_filter.x.set(1, 0, v_y);


        velocity_estimate_filter.velocity = std::sqrt(powf(v_x, 2)
                                                      + powf(v_y, 2));

        // 오차 공분산 계산
        velocity_estimate_filter.P = velocity_estimate_filter.P + velocity_estimate_filter.Q;
    }

    return velocity_estimate_filter.velocity;
}
float debug_sigma = 10;
bool velocity_filter_estimate_state(float gps_v_x, float gps_v_y)
{
    float sigma = debug_sigma;//10; // 값의 유효성 기준, 크면 클 수록 통과하기 좋음
    ModelMatrix innovation = ModelMatrix::zero(2, 1);
    bool result = false;

    if (velocity_estimate_filter.init_flag != 1) {
        return false;
    }
    velocity_estimate_filter.z.set(0, 0, gps_v_x);
    velocity_estimate_filter.z.set(1, 0, gps_v_y);

    // 측정 값과 예측 값 차이
    innovation = velocity_estimate_filter.z - velocity_estimate_filter.H * velocity_estimate_filter.x;

    if (velocity_filter_valid_gate(innovation, velocity_estimate_filter.H, velocity_estimate_filter.R, sigma)) {
        // chi square 기준치 통과
        estimate(velocity_estimate_filter.z);
        result = true;
    } else {
        // chi square 기준치 미달
        result = false;
    }
    return result;
}

float estimate(ModelMatrix z)
{
    velocity_estimate_filter.K = velocity_estimate_filter.P * velocity_estimate_filter.S_inv;

    velocity_estimate_filter.x = velocity_estimate_filter.x +
                                            velocity_estimate_filter.K * (z - velocity_estimate_filter.x);
    velocity_estimate_filter.P = velocity_estimate_filter.P - velocity_estimate_filter.K * velocity_estimate_filter.P;
    velocity_estimate_filter.velocity = std::sqrt(pow(velocity_estimate_filter.x.get(0, 0), 2)
                                                      + pow(velocity_estimate_filter.x.get(1, 0), 2));
    return velocity_estimate_filter.velocity;
}

bool velocity_filter_valid_gate(ModelMatrix innovation, ModelMatrix H, ModelMatrix R, float sigma)
{
    // innovation 공분산 계산 후
    velocity_estimate_filter.S_inv = (H * velocity_estimate_filter.P * H.transpose() + R).inverse();
    ModelMatrix temp = velocity_estimate_filter.S_inv;

    // Chi-Square Statistic 계산
    velocity_estimate_filter.chi_square_value = (innovation.transpose() * velocity_estimate_filter.S_inv * innovation).get(0, 0);
    return velocity_estimate_filter.chi_square_value <= (sigma * sigma);
}
