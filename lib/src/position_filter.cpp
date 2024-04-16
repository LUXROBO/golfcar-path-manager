#include "position_filter.h"
#include "model_matrix_double.h"
#include <cmath>

typedef struct position_filter_context_
{
    // ModelMatrix_D A;  /** 상태 전이 함수 */
    ModelMatrix_D H;          /** 측정 상태 공간 방정식 */
    ModelMatrix_D P;          /** 측정 에러 공분산 */
    ModelMatrix_D predict_P;  /** 측정 에러 예측 공분산 */
    ModelMatrix_D estimate_P;  /** 측정 에러 측정 공분산 */
    ModelMatrix_D K;          /** 칼만 게인 */
    ModelMatrix_D x;          /** state */
    ModelMatrix_D predict_x;  /** 예측 state */
    ModelMatrix_D estimate_x;  /** 측정 state */
    ModelMatrix_D Q;          /** 예측 노이즈 */
    ModelMatrix_D R;          /** 측정 노이즈 */

    pt_control_state_t predict_state;

    std::vector<ModelMatrix_D> v_estimate_buf;
    uint32_t v_estimate_buf_max_size;

    std::vector<ModelMatrix_D> x_residual_buf;
    uint32_t x_residual_buf_max_size;

    std::vector<double> yaw_v_estimate_buf;
    uint32_t yaw_v_estimate_buf_max_size;

    double yaw_H;
    double yaw_P;
    double yaw_K;
    double yaw;
    double yaw_Q;
    double yaw_R;

    bool init_flag;
} position_filter_context_t;

position_filter_context_t position_estimate_filter;

const double W = 2.18;

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
static double R_gps_quality1_data[4] = {1,0,
                                        0,1};   /** gps fix*/
static double R_gps_quality2_data[4] = {0.06231, 0.0788,
                                        0.0788, 0.298146};   /** gps differintial fix*/
static double R_gps_quality4_data[4] = {0.000000233   ,-0.000000014,
                                          -0.000000017  ,-0.00000026};   /** rtk fix*/
static double R_gps_quality5_data[4] = {0.00021676   , -0.00000211,
                                          - 0.00000211  ,0.001087};   /** rtk float*/

static ModelMatrix_D R_gps_xy[6] = {
    ModelMatrix_D(2, 2),
    ModelMatrix_D(2, 2, R_gps_quality1_data),
    ModelMatrix_D(2, 2, R_gps_quality2_data),
    ModelMatrix_D(2, 2),
    ModelMatrix_D(2, 2, R_gps_quality4_data),
    ModelMatrix_D(2, 2, R_gps_quality5_data),
};

/**
 * @brief
 *
 * @param x0
 * @param input
 * @return ModelMatrix_D
 */
static ModelMatrix_D state_equation_jacobi(ModelMatrix_D x0, ModelMatrix_D input);

/**
 * @brief
 *
 * @param z
 * @return pt_control_state_t
 */
static pt_control_state_t estimate(pt_control_state_t z);

bool position_filter_init()
{
    // @todo
    position_estimate_filter.v_estimate_buf_max_size = 16;
    position_estimate_filter.yaw_v_estimate_buf_max_size = 16;
    position_estimate_filter.yaw_P = 1;
    return true;
}

bool position_filter_is_init()
{
    return position_estimate_filter.init_flag;
}

void position_filter_set_position(pt_control_state_t position)
{
    position_estimate_filter.predict_state = position;
    position_estimate_filter.init_flag = true;
}

pt_control_state_t position_filter_get_position()
{
    return position_estimate_filter.predict_state;
}

void position_filter_set_xy(pt_control_state_t position)
{
    position_estimate_filter.predict_state.x = position.x;
    position_estimate_filter.predict_state.y = position.y;
    position_estimate_filter.init_flag = true;
}

pt_control_state_t position_filter_get_xy()
{
    return position_estimate_filter.predict_state;
}

void position_filter_set_yaw(double yaw)
{
    position_estimate_filter.yaw = yaw;
    position_estimate_filter.init_flag = true;
}

double position_filter_get_yaw()
{
    return position_estimate_filter.yaw;
}

bool position_filter_is_init_xy()
{
    return position_estimate_filter.init_flag;
}

void position_filter_set_yaw_R(double yaw_R)
{
    position_estimate_filter.yaw_R = yaw_R;
}

void position_filter_set_yaw_Q(double yaw_Q)
{
    position_estimate_filter.yaw_Q = yaw_Q;
}


ModelMatrix_D state_equation_jacobi(ModelMatrix_D x0, ModelMatrix_D input)
{
    /*
    x = [x,             input = [v,
         y,                      steer(slip_angle)]
         yaw]                    yaw

    x = x + v * dt * cos(yaw + steer)
    y = y + v * dt * sin(yaw + steer)
    yaw = yaw + v * dt * cos(steer) * tan(steer) / W(wheel base);

    jacobian state = [x, y, yaw, v, steer]

    jacobian matrix =
    [
        1,      0,      -v * dt * sin(yaw + steer), dt * cos(yaw + steer),                      -v * dt * sin(yaw + steer),
        0,      1,      v * dt * cos(yaw + steer),  dt * sin(yaw + steer),                      v * dt * cos(yaw + steer),
        0,      0,      1                           dt * cos(steer) * tan(steer) / W,           -v * dt * sin(steer) * tan(steer) / W + v * dt / cos(steer) / W
        0,      0,      0,                          1,                                          0,
        0,      0,      0,                          0,                                          1,
    ]
    */
    // ModelMatrix_D jacobian = ModelMatrix_D::zero(5, 5);
    ModelMatrix_D jacobian = ModelMatrix_D::zero(x0.row(), x0.row());
    double v = input.get(0, 0);
    double yaw = input.get(2, 0);
    double steer = input.get(1, 0);
    double yaw_steer = path_tracker::pi_to_pi(yaw + steer);
    double dt = input.get(3, 0);

    jacobian.set(0, 0, 1.0);
    jacobian.set(0, 2, -1 * v * dt * std::sin(yaw_steer));
    // jacobian.set(0, 3, dt * (std::cos(yaw_steer)));
    // jacobian.set(0, 4, -1 * input.get(0, 0) * dt * std::sin(yaw_steer));

    jacobian.set(1, 1, 1.0);
    jacobian.set(1, 2, v * dt * std::cos(yaw_steer));
    // jacobian.set(1, 3, dt * (std::sin(yaw_steer)));
    // jacobian.set(1, 4, input.get(0, 0) * dt * std::cos(yaw_steer));

    // jacobian.set(2, 2, 1);
    // jacobian.set(2, 3, dt * std::cos(steer) * tan(steer) / W);
    // jacobian.set(2, 3, -1 * input.get(0, 0) * dt * std::sin(steer) * tan(steer) / W
    //                    + input.get(0, 0) * dt / std::cos(steer) / W);

    // jacobian.set(3, 3, 1);

    // jacobian.set(4, 4, 1);

    return jacobian;
}

pt_control_state_t position_filter_predict_xy(double v, double steer, double yaw, double dt)
{
    if (position_estimate_filter.init_flag) {
        double yaw_steer = yaw + steer;
        double temp_input[4] = {v, steer, yaw, dt};
        ModelMatrix_D input = ModelMatrix_D(4, 1, temp_input);
        ModelMatrix_D A = state_equation_jacobi(position_estimate_filter.predict_x, input);

        position_estimate_filter.predict_x.set(0, 0, position_estimate_filter.predict_x.get(0, 0) + v * dt * std::cos(yaw_steer));
        position_estimate_filter.predict_x.set(1, 0, position_estimate_filter.predict_x.get(1, 0) + v * dt * std::sin(yaw_steer));
        position_estimate_filter.predict_P = A * position_estimate_filter.predict_P * A.transpose() + position_estimate_filter.Q;
    }
    position_estimate_filter.predict_state.x = position_estimate_filter.predict_x.get(0, 0);
    position_estimate_filter.predict_state.y = position_estimate_filter.predict_x.get(1, 0);

    return position_estimate_filter.predict_state;
}

pt_control_state_t position_filter_estimate_xy_with_gps(pt_control_state_t gps_pos, int quality)
{
    return estimate(gps_pos);
}

pt_control_state_t estimate(pt_control_state_t estimate_value)
{
    if (position_estimate_filter.init_flag) {
        double temp_estimate_value[2] = {estimate_value.x, estimate_value.y};
        ModelMatrix_D z = ModelMatrix_D(2, 1, temp_estimate_value);
        ModelMatrix_D C_hat = ModelMatrix_D::zero(2, 2);
        ModelMatrix_D C_x_hat = ModelMatrix_D::zero(2, 2);
        position_estimate_filter.K = position_estimate_filter.predict_P * position_estimate_filter.H.transpose() * (((position_estimate_filter.H * position_estimate_filter.predict_P * position_estimate_filter.H.transpose()) + position_estimate_filter.R).inverse());

        position_estimate_filter.estimate_x = position_estimate_filter.predict_x + position_estimate_filter.K * (z - position_estimate_filter.H * position_estimate_filter.predict_x);
        position_estimate_filter.estimate_P = position_estimate_filter.predict_P - position_estimate_filter.K * position_estimate_filter.H * position_estimate_filter.predict_P;

        ModelMatrix_D v_estimate = z - position_estimate_filter.estimate_x;
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

        position_estimate_filter.predict_state.x = position_estimate_filter.x.get(0, 0);
        position_estimate_filter.predict_state.y = position_estimate_filter.x.get(1, 0);

    }
    return position_estimate_filter.predict_state;
}

pt_control_state_t position_filter_predict_yaw(double angular_velocity, double dt)
{
    position_estimate_filter.predict_state.yaw = path_tracker::pi_to_pi(position_estimate_filter.predict_state.yaw + angular_velocity * dt);
    position_estimate_filter.yaw_P = position_estimate_filter.yaw_P + position_estimate_filter.yaw_Q;

    return position_estimate_filter.predict_state;
}

double position_filter_estimate_yaw(double z)
{
    double C_hat = 0;
    position_estimate_filter.yaw_K = position_estimate_filter.yaw_P / (position_estimate_filter.yaw_P + position_estimate_filter.yaw_R);
    position_estimate_filter.predict_state.yaw = position_estimate_filter.predict_state.yaw + position_estimate_filter.yaw_K * (z - position_estimate_filter.predict_state.yaw);
    position_estimate_filter.yaw_P = position_estimate_filter.yaw_P - position_estimate_filter.yaw_K * position_estimate_filter.yaw_P;

    if (position_estimate_filter.yaw_P == 0) {
        position_estimate_filter.yaw_P = 0.0001;
    }
    double v_estimate = z - position_estimate_filter.predict_state.yaw;
    if (position_estimate_filter.yaw_v_estimate_buf.size() >= position_estimate_filter.yaw_v_estimate_buf_max_size) {
        position_estimate_filter.yaw_v_estimate_buf.erase(position_estimate_filter.yaw_v_estimate_buf.begin());
    }
    position_estimate_filter.yaw_v_estimate_buf.push_back(v_estimate);
    for (int i = 0; i < position_estimate_filter.yaw_v_estimate_buf.size(); i++) {
        C_hat = C_hat + position_estimate_filter.yaw_v_estimate_buf[i] * position_estimate_filter.yaw_v_estimate_buf[i];
    }
    position_estimate_filter.yaw_R = C_hat / position_estimate_filter.yaw_v_estimate_buf.size() + position_estimate_filter.yaw_P;

    return position_estimate_filter.predict_state.yaw;
}
