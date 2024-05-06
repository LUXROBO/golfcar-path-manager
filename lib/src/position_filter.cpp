#include "position_filter.h"
#include "model_matrix.h"
#include <cmath>
#include <iostream>

typedef struct position_filter_context_
{
    // ModelMatrix A;       /** 상태 전이 함수 */
    ModelMatrix H;          /** 측정 상태 공간 방정식 */
    ModelMatrix P;          /** 측정 에러 공분산 */
    ModelMatrix predict_P;  /** 측정 에러 예측 공분산 */
    ModelMatrix estimate_P;  /** 측정 에러 측정 공분산 */
    ModelMatrix K;          /** 칼만 게인 */
    ModelMatrix x;          /** state */
    ModelMatrix predict_x;  /** 예측 state */
    ModelMatrix estimate_x;  /** 측정 state */
    ModelMatrix Q;          /** 예측 노이즈 */
    ModelMatrix R;          /** 측정 노이즈 */
    ModelMatrix S_inv;      /** 칼만 게인 계산용 */
    ModelMatrix z;

    pt_control_state_t predict_state;

    std::vector<ModelMatrix> v_estimate_buf;
    uint32_t v_estimate_buf_max_size;

    std::vector<ModelMatrix> x_residual_buf;
    uint32_t x_residual_buf_max_size;

    std::vector<float> yaw_v_estimate_buf;
    uint32_t yaw_v_estimate_buf_max_size;

    uint8_t init_flag;
} position_filter_context_t;

position_filter_context_t position_estimate_filter;

const float W = 2.18;
const float state_member = 6;

float H_array_quality0[30] = {1, 0, 0, 0, 0, 0,
                             0, 0, 1, 0, 0, 0,
                             0, 1, 0, 1, 0, 0,
                             0, 0, 0, 0, 1, 0,
                             0, 0, 0, 0, 0, 1};

float H_array_quality1[24] = {1, 0, 0, 0, 0, 0,
                              0, 0, 1, 0, 0, 0,
                              0, 0, 0, 0, 1, 0,
                              0, 0, 0, 0, 0, 1};

float H_array_quality2[18] = {0, 0, 1, 0, 0, 0,
                             0, 0, 0, 0, 1, 0,
                             0, 0, 0, 0, 0, 1};

float H_array_quality3[6] = {0, 0, 1, 0, 0, 0};


float R_array_quality0[25] = {0.0204, 0.0, 0.0012, 0.0, 0.0,
                              0.0   , 0.0, 0.    , 0.0, 0.0,
                              0.0012, 0.0, 0.0094, 0.0, 0.0,
                              0.0   , 0.0, 0.0   , 0.0, 0.0,
                              0.0   , 0.0, 0.0   , 0.0, 0.0};

float R_array_quality1[16] = {0.0204, 0, 0, 0,
                               0, 0, 0, 0,
                               0, 0, 0, 0,
                               0, 0, 0, 0};

float R_array_quality2[9] = {0, 0, 0,
                             0, 0, 0,
                             0, 0, 0};

float R_array_quality3[1] = {0};
// float R_array_quality0[25] = {0.01, 0.01, 0.01, 0.01, 0.01,
//                                0.01, 0.01, 0.01, 0.01, 0.01,
//                                0.01, 0.01, 0.01, 0.01, 0.01,
//                                0.01, 0.01, 0.01, 0.01, 0.01,
//                                0.01, 0.01, 0.01, 0.01, 0.01};

// float R_array_quality1[16] = {0.01, 0.01, 0.01, 0.01,
//                                0.01, 0.01, 0.01, 0.01,
//                                0.01, 0.01, 0.01, 0.01,
//                                0.01, 0.01, 0.01, 0.01};

// float R_array_quality2[9] = {0.01, 0.01, 0.01,
//                              0.01, 0.01, 0.01,
//                              0.01, 0.01, 0.01};

// float R_array_quality3[1] = {0.01};


/*
kalman filter innovation -> 측정 값과 실제 값 차이
kalman filter residual ->   예측 값과 측정 값 차이
*/

/*
GPS Quality indicator:
0: Fix not valid
1: GPS fix
2: Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode
3: Not applicable
4: RTK Fixed, xFill
5: RTK Float, OmniSTAR XP/HP, Location RTK, RTX
6: INS Dead reckoning
*/

/**
 * @brief
 * @param x0
 * @param input
 * @return ModelMatrix
 */
static ModelMatrix state_equation_jacobi(ModelMatrix x0, ModelMatrix input);

/**
 * @brief
 *
 * @param z
 * @return pt_control_state_t
 */
static pt_control_state_t estimate(ModelMatrix z);

bool position_filter_init()
{
    // @todo
    position_estimate_filter.v_estimate_buf_max_size = 16;
    // position_estimate_filter.yaw_v_estimate_buf_max_size = 16;
    // position_estimate_filter.yaw_P = 1;

    position_estimate_filter.P = ModelMatrix::identity(state_member, state_member);
    position_estimate_filter.predict_P = ModelMatrix::identity(state_member, state_member);
    position_estimate_filter.estimate_P = ModelMatrix::identity(state_member, state_member);

    position_estimate_filter.x = ModelMatrix::zero(state_member, 1);
    position_estimate_filter.predict_x = ModelMatrix::zero(state_member, 1);
    position_estimate_filter.estimate_x = ModelMatrix::zero(state_member, 1);

    position_estimate_filter.R = ModelMatrix::identity(state_member, state_member);
    position_estimate_filter.Q = ModelMatrix::zero(state_member, state_member);
    position_estimate_filter.K = ModelMatrix::zero(state_member, state_member);
    position_estimate_filter.H = ModelMatrix::identity(state_member, state_member);

    position_estimate_filter.S_inv = ModelMatrix::zero(state_member, state_member);

    float Q_array[36] = {0, 0, 0, 0, 0, 0,
                         0.0, 0.00146, 0.00018, 0.00035, -0.00001, 0.00001,
                         0.0, 0.00018, 0.00046, -0.00034, 0.00001, 0.00001,
                         0.0, 0.00035, -0.00034, 0.00122, -0.00001, 0.00001,
                         0.0, -0.00001,  0.00001, 0.00001, 0.00001, 0.00001,
                         0.0, 0.00001, 0.00001, 0.00001, 0.00001, 0.00017};
    position_estimate_filter.Q = ModelMatrix(6, 6, Q_array);

    float R_array[25] = {1, 0.01, 0.01, 0.01, 0.01,
                          0.01, 1, 0.01, 0.01, 0.01,
                          0.01, 0.01, 1, 0.01, 0.01,
                          0.01, 0.01, 0.01, 1, 0.01,
                          0.01, 0.01, 0.01, 0.01, 1};
    position_estimate_filter.R = ModelMatrix(6, 6, R_array);

    return true;
}

bool position_filter_is_init()
{
    if (position_estimate_filter.init_flag == POSITION_FILTER_INIT_BOTH) {
        return true;
    }
    return false;
}

void position_filter_set_position(pt_control_state_t position)
{
    position_estimate_filter.predict_state = position;
    position_estimate_filter.init_flag |= POSITION_FILTER_INIT_BOTH;
}

// pt_control_state_t position_filter_get_position()
// {
//     return position_estimate_filter.predict_state;
// }

void position_filter_set_xy(float x, float y)
{
    position_estimate_filter.predict_state.x = x;
    position_estimate_filter.predict_state.y = y;
    position_estimate_filter.predict_x.set(4, 0, x);
    position_estimate_filter.predict_x.set(5, 0, y);
    position_estimate_filter.x.set(4, 0, x);
    position_estimate_filter.x.set(5, 0, y);

    position_estimate_filter.init_flag |= POSITION_FILTER_INIT_XY;
}

void position_filter_set_yaw(float yaw)
{
    position_estimate_filter.predict_state.yaw = yaw;
    position_estimate_filter.predict_x.set(3, 0, yaw);
    position_estimate_filter.x.set(3, 0, yaw);
    position_estimate_filter.init_flag |= POSITION_FILTER_INIT_YAW;
}

pt_control_state_t position_filter_get_state()
{
    return position_estimate_filter.predict_state;
}

ModelMatrix position_filter_get_predict_x()
{
    return position_estimate_filter.predict_x;
}

ModelMatrix state_equation_jacobi(ModelMatrix x0, ModelMatrix input)
{
    /*
    x = [v,             input = [v,
         slip,                      steer]
         yaw rate,
         yaw,
         x,
         y]

    x = x + v * dt * cos(yaw + steer)
    y = y + v * dt * sin(yaw + steer)
    yaw = yaw + v * dt * cos(steer) * tan(steer) / W(wheel base);
    */

    ModelMatrix p_x = position_estimate_filter.predict_x;

    ModelMatrix jacobian = ModelMatrix::zero(state_member, state_member);
    float v = input.get(0, 0);
    float steer = input.get(1, 0);
    float dt = input.get(2, 0);

    float pre_v = p_x.get(0, 0);
    float slip = p_x.get(1, 0);
    float yaw = p_x.get(3, 0);

    jacobian.set(0, 0, 1.0);

    jacobian.set(2, 0, std::cos(slip) * std::tan(steer) / W);
    jacobian.set(2, 2, -pre_v * std::sin(slip) * std::tan(steer) / W);

    jacobian.set(3, 2, dt);
    jacobian.set(3, 3, 1);

    jacobian.set(4, 0, dt * std::cos(yaw + slip));
    jacobian.set(4, 1, -pre_v * dt * std::sin(yaw + slip));
    jacobian.set(4, 3, -pre_v * dt * std::sin(yaw + slip));
    jacobian.set(4, 4, 1);

    jacobian.set(5, 0, dt * std::sin(yaw + slip));
    jacobian.set(5, 1, pre_v * dt * std::cos(yaw + slip));
    jacobian.set(5, 3, pre_v * dt * std::cos(yaw + slip));
    jacobian.set(5, 5, 1);

    return jacobian;
}

pt_control_state_t position_filter_predict_state(float v, float steer, float dt)
{
    if (position_filter_is_init()) {
        float temp_input[3] = {v, steer, dt};
        ModelMatrix input = ModelMatrix(3, 1, temp_input);
        ModelMatrix A = state_equation_jacobi(position_estimate_filter.predict_x, input);
        ModelMatrix temp_x = ModelMatrix::zero(6, 1);
        ModelMatrix p_x = position_estimate_filter.predict_x;

        temp_x.set(0, 0, v);
        temp_x.set(1, 0, std::atan(std::tan(steer) / 2));
        temp_x.set(2, 0, p_x.get(0, 0) * std::cos(p_x.get(0, 1)) * std::tan(steer) / W);
        temp_x.set(3, 0, path_tracker::pi_to_pi(p_x.get(3, 0) + dt * p_x.get(2, 0)));
        temp_x.set(4, 0, p_x.get(4, 0) + dt * p_x.get(0, 0) * std::cos(p_x.get(3, 0) + p_x.get(1, 0)));
        temp_x.set(5, 0, p_x.get(5, 0) + dt * p_x.get(0, 0) * std::sin(p_x.get(3, 0) + p_x.get(1, 0)));

        position_estimate_filter.predict_x = temp_x;
        position_estimate_filter.predict_P = A * position_estimate_filter.predict_P * A.transpose() + position_estimate_filter.Q;

        position_estimate_filter.predict_state.x = position_estimate_filter.predict_x.get(4, 0);
        position_estimate_filter.predict_state.y = position_estimate_filter.predict_x.get(5, 0);
        position_estimate_filter.predict_state.yaw = position_estimate_filter.predict_x.get(3, 0);
    }
    return position_estimate_filter.predict_state;
}

pt_control_state_t position_filter_estimate_state(position_filter_z_format_t z_value, int quality)
{
    // z format = [gps v, yaw rate, gps slip+yaw, gps x, gps y]
    float sigma = 0; // 값의 유효성 기준, 크면 클 수록 통과하기 좋음
    ModelMatrix resize_z;
    ModelMatrix innovation;
    position_estimate_filter.z = ModelMatrix::zero(5, 1);
    position_estimate_filter.z.set(0, 0, z_value.gps_v);
    position_estimate_filter.z.set(1, 0, z_value.yaw_rate);
    position_estimate_filter.z.set(2, 0, z_value.gps_yaw);
    position_estimate_filter.z.set(3, 0, z_value.gps_x);
    position_estimate_filter.z.set(4, 0, z_value.gps_y);
    if (quality == POSITION_FILTER_QUALITY_ALL) {
        // no problem
        position_estimate_filter.H = ModelMatrix(5, 6, H_array_quality0);
        position_estimate_filter.R = ModelMatrix(5, 5, R_array_quality0);

        resize_z = ModelMatrix::zero(5, 1);
        resize_z = position_estimate_filter.z;
        sigma = 10;
    } else if (quality == POSITION_FILTER_QUALITY_EXCEPT_YAW) {
        // yaw ds is so low
        position_estimate_filter.H = ModelMatrix(4, 6, H_array_quality1);
        position_estimate_filter.R = ModelMatrix(4, 4, R_array_quality1);

        resize_z = ModelMatrix::zero(4, 1);
        resize_z.set(0, 0, z_value.gps_v);
        resize_z.set(1, 0, z_value.yaw_rate);
        resize_z.set(2, 0, z_value.gps_x);
        resize_z.set(3, 0, z_value.gps_y);
        sigma = 5;
    } else if (quality == POSITION_FILTER_QUALITY_EXCEPT_GPS_DERIVATIVE) {
        // velocity is so low
        position_estimate_filter.H = ModelMatrix(3, 6, H_array_quality2);
        position_estimate_filter.R = ModelMatrix(3, 3, R_array_quality2);

        resize_z = ModelMatrix::zero(3, 1);
        resize_z.set(0, 0, z_value.yaw_rate);
        resize_z.set(1, 0, z_value.gps_x);
        resize_z.set(2, 0, z_value.gps_y);
        sigma = 3;
    } else {
        // it has lots of problems
        float H_array[6] = {0, 0, 1, 0, 0, 0};
        position_estimate_filter.H = ModelMatrix(1, 6, H_array_quality3);
        position_estimate_filter.R = ModelMatrix(1, 1, R_array_quality3);

        resize_z = ModelMatrix::zero(1, 1);
        resize_z.set(0, 0, z_value.yaw_rate);
        sigma = 3;
    }
    innovation = resize_z - position_estimate_filter.H * position_estimate_filter.predict_x;

    // yaw를 사용할 때는 innovation에서 pi to pi로 오차가 커지는 상황을 방지해야함
    if (quality == POSITION_FILTER_QUALITY_ALL) {
        innovation.set(2, 0, path_tracker::pi_to_pi(innovation.get(2, 0)));
    }

    if (position_filter_valid_gate(innovation, position_estimate_filter.H, sigma)) {
        return estimate(resize_z);
    } else {
        return position_estimate_filter.predict_state;
    }
}

pt_control_state_t estimate(ModelMatrix z)
{
    if (position_filter_is_init()) {
        ModelMatrix C_hat = ModelMatrix::zero(state_member, state_member);
        ModelMatrix C_x_hat = ModelMatrix::zero(state_member, state_member);
        ModelMatrix S = ModelMatrix::zero(state_member, state_member);
        position_estimate_filter.K = position_estimate_filter.predict_P * position_estimate_filter.H.transpose() * position_estimate_filter.S_inv;

        position_estimate_filter.estimate_x = position_estimate_filter.predict_x +
                                              position_estimate_filter.K * (z - position_estimate_filter.H * position_estimate_filter.predict_x);
        position_estimate_filter.estimate_P = position_estimate_filter.predict_P - position_estimate_filter.K * position_estimate_filter.H * position_estimate_filter.predict_P;
        ModelMatrix v_estimate = position_estimate_filter.z - position_estimate_filter.estimate_x;
        if (position_estimate_filter.v_estimate_buf.size() >= position_estimate_filter.v_estimate_buf_max_size) {
            position_estimate_filter.v_estimate_buf.erase(position_estimate_filter.v_estimate_buf.begin());
        }
        position_estimate_filter.v_estimate_buf.push_back(v_estimate);
        for (int i = 0; i < position_estimate_filter.v_estimate_buf.size(); i++) {
            C_hat = C_hat + position_estimate_filter.v_estimate_buf[i] * position_estimate_filter.v_estimate_buf[i].transpose();
        }
        position_estimate_filter.R = C_hat / position_estimate_filter.v_estimate_buf.size() + position_estimate_filter.estimate_P;
        position_estimate_filter.x = position_estimate_filter.estimate_x;
        position_estimate_filter.predict_x = position_estimate_filter.estimate_x;
        position_estimate_filter.P = position_estimate_filter.estimate_P;
        position_estimate_filter.predict_P = position_estimate_filter.estimate_P;
        position_estimate_filter.predict_state.yaw = position_estimate_filter.predict_x.get(3, 0);
        position_estimate_filter.predict_state.x = position_estimate_filter.predict_x.get(4, 0);
        position_estimate_filter.predict_state.y = position_estimate_filter.predict_x.get(5, 0);
    }
    return position_estimate_filter.predict_state;
}

bool position_filter_valid_gate(ModelMatrix innovation, ModelMatrix H, float sigma)
{
    // position_estimate_filter.S_inv =
    position_estimate_filter.S_inv = (H * position_estimate_filter.predict_P * H.transpose() + position_estimate_filter.R).inverse();
    ModelMatrix temp = position_estimate_filter.S_inv;

    float V = (innovation.transpose() * position_estimate_filter.S_inv * innovation).get(0, 0);
    return V <= (sigma * sigma);
}
