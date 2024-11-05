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
    pt_control_state_t estimate_state;

    std::vector<ModelMatrix> v_estimate_buf;
    uint32_t v_estimate_buf_max_size;

    std::vector<ModelMatrix> x_residual_buf;
    uint32_t x_residual_buf_max_size;

    std::vector<float> yaw_v_estimate_buf;
    uint32_t yaw_v_estimate_buf_max_size;

    position_filter_init_state_t init_flag;
    float last_update_time;
    float chi_square_v;
} position_filter_context_t;

position_filter_context_t position_estimate_filter;

const float W = 2.18;
const float state_member = 5;


// z format = [gps v, yaw rate, gps slip+yaw, gps x, gps y]
// x format = [v, slip angle, angular v, yaw, x, y]
/*
gps v = v
yaw rate = angular v
gps slip + yaw = slip angle + yaw
gps x = x
gps y = y
*/
static float H_array_quality0[25] = {1, 0, 0, 0, 0,
                                     0, 1, 0, 0, 0,
                                     0, 0, 1, 0, 0,
                                     0, 0, 0, 1, 0,
                                     0, 0, 0, 0, 1}; /** */

// static float H_array_quality1[20] = {1, 0, 0, 0, 0,
//                                      0, 1, 0, 0, 0,
//                                      0, 0, 0, 1, 0,
//                                      0, 0, 0, 0, 1};

// static float H_array_quality2[18] = {0, 0, 1, 0, 0, 0,
//                              0, 0, 0, 0, 1, 0,
//                              0, 0, 0, 0, 0, 1};

static float H_array_quality3[5] = {0, 1, 0, 0, 0};

static float H_array_imu_ins[10] = {0, 1, 0, 0, 0,
                                    0, 0, 1, 0, 0};


// Z 모두
// 현재 대각 값 외에는 설정하지 않음
static float R_array_quality0[25] = {0.1, 0.0,    0.0,   0.0,   0.0,    // gps velocity -> 정확도가 높지 않음
                                     0.0, 0.0001, 0.0,   0.0,   0.0,    // yaw rate -> imu로 정확도가 높음
                                     0.0, 0.0,    0.002, 0.0,   0.0,    // gps yaw (yaw + slip) -> gps quality가 높을 경우 정확도가 올라감)
                                     0.0, 0.0,    0.0,   0.0003, 0.0,    // gps x
                                     0.0, 0.0,    0.0,   0.0,   0.0003}; // gps y
static float R_array_quality0_float[25] = {0.1, 0.0,    0.0,   0.0,   0.0,    // gps velocity -> 정확도가 높지 않음
                                           0.0, 0.01, 0.0,   0.0,   0.0,    // yaw rate -> imu로 정확도가 높음
                                           0.0, 0.0,    0.25, 0.0,   0.0,    // gps yaw (yaw + slip) -> gps quality가 높을 경우 정확도가 올라감)
                                           0.0, 0.0,    0.0,   0.3, 0.0,    // gps x
                                           0.0, 0.0,    0.0,   0.0,   0.3}; // gps y
static float R_array_quality0_dgps[25] = {10, 0.0,    0.0,   0.0,   0.0,    // gps velocity -> 정확도가 높지 않음
                                          0.0, 10, 0.0,   0.0,   0.0,    // yaw rate -> imu로 정확도가 높음
                                          0.0, 0.0, 10, 0.0,   0.0,    // gps yaw (yaw + slip) -> gps quality가 높을 경우 정확도가 올라감)
                                          0.0, 0.0, 0.0,   10, 0.0,    // gps x
                                          0.0, 0.0, 0.0,   0.0,   10}; // gps y


// Z중 YAW 제외
static float R_array_quality1[16] = {0.1, 0, 0, 0,
                               0, 0.0001, 0, 0,
                               0, 0, 0.0001, 0,
                               0, 0, 0, 0.0001};

// Z중 V, YAW 제외
static float R_array_quality2[9] = {0, 0, 0,
                             0, 0, 0,
                             0, 0, 0};

static float R_array_quality3[1] = {0.00001};

static float R_array_quality_imu_ins[4] = {0.00001, 0,
                                           0,       0.01};

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

    // float Q_array[36] = {0, 0, 0, 0, 0, 0,
    //                      0.0, 0.00146, 0.00018, 0.00035, -0.00001, 0.00001,
    //                      0.0, 0.00018, 0.00046, -0.00034, 0.00001, 0.00001,
    //                      0.0, 0.00035, -0.00034, 0.00122, -0.00001, 0.00001,
    //                      0.0, -0.00001,  0.00001, 0.00001, 0.00001, 0.00001,
    //                      0.0, 0.00001, 0.00001, 0.00001, 0.00001, 0.00017};
    float Q_array[25] = {0.000001, 0       , 0        , 0       , 0,
                         0       , 0.000178, 0.0      , 0.0     , 0.0,
                         0       , 0       , 0.0001   , 0.0     , 0.0,
                         0       , 0       , 0.0      , 0.00042 , 0.0,
                         0       , 0       , 0.0      , 0.0     , 0.00085};
    position_estimate_filter.Q = ModelMatrix(state_member, state_member, Q_array);

    position_estimate_filter.R = ModelMatrix(5, 5, R_array_quality0);

    return true;
}

bool position_filter_is_init()
{
    if (position_estimate_filter.init_flag == position_filter_init_both) {
        return true;
    }
    return false;
}

position_filter_init_state_t position_filter_get_init_state()
{
    return position_estimate_filter.init_flag;
}

void position_filter_set_R(int gps_quality)
{
    switch (gps_quality) {
        case 4:
            position_estimate_filter.R = ModelMatrix(5, 5, R_array_quality0);
            break;
        case 5:
            position_estimate_filter.R = ModelMatrix(5, 5, R_array_quality0_float);
            break;
        default:
            position_estimate_filter.R = ModelMatrix(5, 5, R_array_quality0_dgps);
            break;
    }
    // 효율적으로 R값 변경 방법 고민
}

// void position_filter_set_R(float R_value, int mode)
// {
//     switch (mode) {
//     case 0: // RTK Quality 4 -> RTK fixed
//         //  어떤 값이든 기존 R로 롤백
//         position_estimate_filter.R = ModelMatrix(5, 5, R_array_quality0);
//         break;
//     case 1: // RTK Quality 5 -> RTK float
//         position_estimate_filter.R = ModelMatrix(5, 5, R_array_quality0);

//         position_estimate_filter.R.set(3, 3, R_value);
//         position_estimate_filter.R.set(4, 4, R_value);
//         break;
//     }
// }

void position_filter_set_position(pt_control_state_t position)
{
    position_estimate_filter.predict_state = position;
    position_estimate_filter.estimate_state = position;
    position_estimate_filter.predict_x.set(0, 0, position.v);
    position_estimate_filter.predict_x.set(2, 0, position.yaw);
    position_estimate_filter.predict_x.set(3, 0, position.x);
    position_estimate_filter.predict_x.set(4, 0, position.y);
    position_estimate_filter.init_flag = position_filter_init_both;
}

// pt_control_state_t position_filter_get_position()
// {
//     return position_estimate_filter.predict_state;
// }

void position_filter_set_xy(double x, double y)
{
    position_estimate_filter.predict_state.x = x;
    position_estimate_filter.predict_state.y = y;
    position_estimate_filter.estimate_state.x = x;
    position_estimate_filter.estimate_state.y = y;
    position_estimate_filter.predict_x.set(3, 0, x);
    position_estimate_filter.predict_x.set(4, 0, y);
    position_estimate_filter.x.set(3, 0, x);
    position_estimate_filter.x.set(4, 0, y);

    if ((position_estimate_filter.init_flag == position_filter_init_yaw) ||
        (position_estimate_filter.init_flag == position_filter_init_both)) {
        position_estimate_filter.init_flag = position_filter_init_both;
    } else {
        position_estimate_filter.init_flag = position_filter_init_xy;
    }
}

void position_filter_set_last_update_time(float update_time)
{
    position_estimate_filter.last_update_time = update_time;
}

void position_filter_set_yaw(float yaw)
{
    position_estimate_filter.predict_state.yaw = yaw;
    position_estimate_filter.estimate_state.yaw = yaw;

    position_estimate_filter.predict_x.set(2, 0, yaw);
    position_estimate_filter.x.set(2, 0, yaw);

    if ((position_estimate_filter.init_flag == position_filter_init_xy) ||
        (position_estimate_filter.init_flag == position_filter_init_both)) {
        position_estimate_filter.init_flag = position_filter_init_both;
    } else {
        position_estimate_filter.init_flag = position_filter_init_yaw;
    }
}

pt_control_state_t position_filter_get_state()
{
    return position_estimate_filter.predict_state;
}

ModelMatrix position_filter_get_predict_x()
{
    return position_estimate_filter.predict_x;
}

float position_filter_get_chi_square_value()
{
    return position_estimate_filter.chi_square_v;
}

void position_filter_set_predict_x(ModelMatrix new_x)
{
    position_estimate_filter.predict_x = new_x;
}

ModelMatrix state_equation_jacobi(ModelMatrix x0, ModelMatrix input)
{

    /*
    x = [v,                     input = [v,
         slip,                           steer,
         yaw rate,                       dt]
         yaw,
         x,
         y]

        v = v
        slip = atan(tan(steer) / 2)
        yaw_rate = pre_v * cos(pre_slip) * tan(steer) / W
        yaw = pre_yaw + dt * pre_yaw_rate
        x = pre_x + pre_v * cos(pre_yaw + pre_slip)
        x = pre_y + pre_v * sin(pre_yaw + pre_slip)
        */

    ModelMatrix p_x = position_estimate_filter.predict_x;

    ModelMatrix jacobian = ModelMatrix::zero(state_member, state_member);
    float v = input.get(0, 0);
    float steer = input.get(1, 0);
    float slip = input.get(2, 0);
    float dt = input.get(3, 0);

    float pre_v = p_x.get(0, 0);
    // float slip = p_x.get(1, 0);
    float yaw = p_x.get(3, 0);

    // v
    jacobian.set(0, 0, 0.0);

    // w
    jacobian.set(1, 0, std::cos(slip) * std::tan(steer) / W);

    // yaw
    jacobian.set(2, 1, dt);
    jacobian.set(2, 2, 1);

    // x
    jacobian.set(3, 0, dt * std::cos(yaw + slip));
    jacobian.set(3, 2, -pre_v * dt * std::sin(yaw + slip));
    jacobian.set(3, 3, 1);

    // y
    jacobian.set(4, 0, dt * std::sin(yaw + slip));
    jacobian.set(4, 2, pre_v * dt * std::cos(yaw + slip));
    jacobian.set(4, 4, 1);

    return jacobian;
}

pt_control_state_t position_filter_predict_state(float v, float steer, float pitch, float updated_time)
{
    if (position_filter_is_init()) {
        float dt = updated_time - position_estimate_filter.last_update_time;
        float slip_angle = std::atan(std::tan(steer) / 2);
        float temp_input[4] = {v, steer, slip_angle, dt};
        position_estimate_filter.last_update_time = updated_time;

        ModelMatrix input = ModelMatrix(4, 1, temp_input);
        ModelMatrix A = state_equation_jacobi(position_estimate_filter.predict_x, input);
        ModelMatrix temp_x = ModelMatrix::zero(5, 1);
        ModelMatrix p_x = position_estimate_filter.predict_x;

        /*
        x = [v,
            slip,
            yaw rate,
            yaw,
            x,
            y]

        v = v
        slip = atan(tan(steer) / 2)
        yaw_rate = pre_v * cos(pre_slip) * tan(steer) / W
        yaw = pre_yaw + dt * pre_yaw_rate
        x = pre_x + pre_v * cos(pre_yaw + pre_slip)
        x = pre_y + pre_v * sin(pre_yaw + pre_slip)
        */

        // kinematic 식 계산
        temp_x.set(0, 0, v);
        temp_x.set(1, 0, p_x.get(0, 0) * std::cos(slip_angle) * std::tan(steer) / W);
        temp_x.set(2, 0, path_tracker::pi_to_pi(p_x.get(2, 0) + dt * p_x.get(1, 0)));
        temp_x.set(3, 0, p_x.get(3, 0) + dt * p_x.get(0, 0) * std::cos(pitch) * std::cos(p_x.get(2, 0) + slip_angle));
        temp_x.set(4, 0, p_x.get(4, 0) + dt * p_x.get(0, 0) * std::cos(pitch) * std::sin(p_x.get(2, 0) + slip_angle));

        position_estimate_filter.predict_x = temp_x;

        // 오차 공분산 계산
        position_estimate_filter.predict_P = A * position_estimate_filter.predict_P * A.transpose() + position_estimate_filter.Q;
        // position_estimate_filter.predict_P = position_estimate_filter.predict_P + position_estimate_filter.Q;

        // double theta1 = std::atan2(position_estimate_filter.predict_x.get(3, 0), position_estimate_filter.predict_x.get(4, 0));
        // double theta2 = position_estimate_filter.predict_state.yaw - theta1;
        // double L1 = std::sqrt(powf(position_estimate_filter.predict_x.get(3, 0), 2) +
        //                    powf(position_estimate_filter.predict_x.get(4, 0), 2));

        // double delta_x_w = std::cos(theta2) * L1;
        // double delta_y_w = std::sin(theta2) * L1;

        position_estimate_filter.predict_state.v = v;
        position_estimate_filter.predict_state.steer = steer;
        // position_estimate_filter.predict_state.x += delta_x_w;
        // position_estimate_filter.predict_state.y += delta_y_w;
        position_estimate_filter.predict_state.x = position_estimate_filter.predict_x.get(3, 0);
        position_estimate_filter.predict_state.y = position_estimate_filter.predict_x.get(4, 0);
        position_estimate_filter.predict_state.yaw = position_estimate_filter.predict_x.get(2, 0);
    }
    return position_estimate_filter.predict_state;
}

bool position_filter_estimate_state(position_filter_z_format_t z_value, int quality)
{
    // z format = [gps v, yaw rate, gps slip+yaw, gps x, gps y]
    float sigma = 0; // 값의 유효성 기준, 크면 클 수록 통과하기 좋음
    ModelMatrix resize_z;
    ModelMatrix innovation;
    ModelMatrix temp_R;
    position_estimate_filter.z = ModelMatrix::zero(5, 1);
    position_estimate_filter.z.set(0, 0, z_value.gps_v);
    position_estimate_filter.z.set(1, 0, z_value.yaw_rate);
    position_estimate_filter.z.set(2, 0, z_value.gps_yaw);
    position_estimate_filter.z.set(3, 0, z_value.gps_x);
    position_estimate_filter.z.set(4, 0, z_value.gps_y);
    if (quality == POSITION_FILTER_QUALITY_ALL) {
        position_estimate_filter.H = ModelMatrix(5, 5, H_array_quality0);
        temp_R = position_estimate_filter.R;

        resize_z = ModelMatrix::zero(5, 1);
        resize_z = position_estimate_filter.z;
        sigma = 4;

        if (fabsf(z_value.gps_yaw - position_estimate_filter.predict_x.get(2, 0)) > PT_M_PI) {
            if (position_estimate_filter.predict_x.get(2, 0) > 0) {
                z_value.gps_yaw += PT_M_PI * 2;
            } else {
                z_value.gps_yaw -= PT_M_PI * 2;
            }
            resize_z.set(2, 0, z_value.gps_yaw);
        }
    } else if (quality == POSITION_FILTER_QUALITY_ONLY_IMU_WITH_YAW) {
        position_estimate_filter.H = ModelMatrix(2, 5, H_array_imu_ins);
        position_estimate_filter.R = ModelMatrix(2, 2, R_array_quality_imu_ins);

        resize_z = ModelMatrix::zero(2, 1);
        resize_z.set(0, 0, z_value.yaw_rate);
        resize_z.set(1, 0, z_value.gps_yaw);
        sigma = 1.5;

        if (fabsf(z_value.gps_yaw - position_estimate_filter.predict_x.get(2, 0)) > PT_M_PI) {
            if (position_estimate_filter.predict_x.get(2, 0) > 0) {
                z_value.gps_yaw += PT_M_PI * 2;
            } else {
                z_value.gps_yaw -= PT_M_PI * 2;
            }
            resize_z.set(1, 0, z_value.gps_yaw);
        }
    } else {
        // imu setting
        float H_array[5] = {0, 1, 0, 0, 0};
        position_estimate_filter.H = ModelMatrix(1, 5, H_array_quality3);
        position_estimate_filter.R = ModelMatrix(1, 1, R_array_quality3);

        temp_R = ModelMatrix(1,1);
        temp_R.set(0, 0, position_estimate_filter.R.get(1, 1));

        resize_z = ModelMatrix::zero(1, 1);
        resize_z.set(0, 0, z_value.yaw_rate);

        sigma = 1.7;
    }
    // 측정 값과 예측 값 차이
    innovation = resize_z - position_estimate_filter.H * position_estimate_filter.predict_x;

    // yaw를 사용할 때는 innovation에서 pi to pi로 오차가 커지는 상황을 방지해야함
    if (quality == POSITION_FILTER_QUALITY_ALL) {
        innovation.set(1, 0, path_tracker::pi_to_pi(innovation.get(1, 0)));
    }

    if (position_filter_valid_gate(innovation, position_estimate_filter.H, position_estimate_filter.R, sigma)) {
        // chi square 기준치 통과
        estimate(resize_z);
        return true;
    } else {
        // chi square 기준치 미달
        return false;
    }

    return true;
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
        // ModelMatrix v_estimate = position_estimate_filter.z - position_estimate_filter.estimate_x;
        // if (position_estimate_filter.v_estimate_buf.size() >= position_estimate_filter.v_estimate_buf_max_size) {
        //     position_estimate_filter.v_estimate_buf.erase(position_estimate_filter.v_estimate_buf.begin());
        // }

        // position_estimate_filter.v_estimate_buf.push_back(v_estimate);
        // for (int i = 0; i < position_estimate_filter.v_estimate_buf.size(); i++) {
        //     C_hat = C_hat + position_estimate_filter.v_estimate_buf[i] * position_estimate_filter.v_estimate_buf[i].transpose();
        // }

        // position_estimate_filter.R = C_hat / position_estimate_filter.v_estimate_buf.size() + position_estimate_filter.estimate_P;
        position_estimate_filter.estimate_x.set(2, 0, path_tracker::pi_to_pi(position_estimate_filter.estimate_x.get(2, 0)));



        // 전 yaw 기준으로 yaw 변화량
        // double theta1 = std::atan2(position_estimate_filter.estimate_x.get(3, 0), position_estimate_filter.estimate_x.get(4, 0));
        // double theta2 = position_estimate_filter.estimate_state.yaw - theta1;
        // double L1 = std::sqrt(powf(position_estimate_filter.estimate_x.get(3, 0), 2) +
        //                    powf(position_estimate_filter.estimate_x.get(4, 0), 2));
        // double delta_x_w = std::cos(theta2) * L1;
        // double delta_y_w = std::sin(theta2) * L1;


        position_estimate_filter.x = position_estimate_filter.estimate_x;
        position_estimate_filter.predict_x = position_estimate_filter.estimate_x;
        position_estimate_filter.P = position_estimate_filter.estimate_P;
        position_estimate_filter.predict_P = position_estimate_filter.estimate_P;

        position_estimate_filter.estimate_state.yaw = position_estimate_filter.predict_x.get(2, 0);
        position_estimate_filter.estimate_state.x = position_estimate_filter.predict_x.get(3, 0);
        position_estimate_filter.estimate_state.y = position_estimate_filter.predict_x.get(4, 0);
        // position_estimate_filter.estimate_state.x += delta_x_w;
        // position_estimate_filter.estimate_state.y += delta_y_w;

        position_estimate_filter.predict_state = position_estimate_filter.estimate_state;
    }
    return position_estimate_filter.predict_state;
}

bool position_filter_valid_gate(ModelMatrix innovation, ModelMatrix H, ModelMatrix R, float sigma)
{
    // innovation 공분산 계산 후
    position_estimate_filter.S_inv = (H * position_estimate_filter.predict_P * H.transpose() + R).inverse();
    ModelMatrix temp = position_estimate_filter.S_inv;

    // Chi-Square Statistic 계산
    position_estimate_filter.chi_square_v = (innovation.transpose() * position_estimate_filter.S_inv * innovation).get(0, 0);
    return position_estimate_filter.chi_square_v <= (sigma * sigma);
}
