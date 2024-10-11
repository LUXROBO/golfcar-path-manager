#include "velocity_filter.h"
#include "model_matrix.h"
#include <cmath>
#include <iostream>

typedef struct velocity_filter_context_
{
    // float A;       /** 상태 전이 함수 */
    float H;          /** 측정 상태 공간 방정식 */
    float P;          /** 측정 에러 공분산 */
    float predict_P;  /** 측정 에러 예측 공분산 */
    float estimate_P;  /** 측정 에러 측정 공분산 */
    float K;          /** 칼만 게인 */
    float x;          /** state */
    float predict_x;  /** 예측 state */
    float estimate_x;  /** 측정 state */
    float Q;          /** 예측 노이즈 */
    float R;          /** 측정 노이즈 */
    float S_inv;      /** 칼만 게인 계산용 */
    float z;

    std::vector<float> v_estimate_buf;
    uint32_t v_estimate_buf_max_size;

    std::vector<float> x_residual_buf;
    uint32_t x_residual_buf_max_size;

    std::vector<float> yaw_v_estimate_buf;
    uint32_t yaw_v_estimate_buf_max_size;

    float last_update_time;
    int init_flag;
} velocity_filter_context_t;

velocity_filter_context_t velocity_estimate_filter;


static float H_array_quality0 = 1; /** */

static float H_array_quality1 = 1;

static float H_array_quality2 = 1;

static float H_array_quality3 = 1;


// Z 모두
// 현재 대각 값 외에는 설정하지 않음
static float R_array_quality0 = 1;

// float R_array_quality0[25] = {0.0204, 0.0, 0.0012, 0.0, 0.0,
//                               0.0   , 0.0, 0.    , 0.0, 0.0,
//                               0.0012, 0.0, 0.0094, 0.0, 0.0,
//                               0.0   , 0.0, 0.0   , 0.0, 0.0,
//                               0.0   , 0.0, 0.0   , 0.0, 0.0};

// Z중 YAW 제외
static float R_array_quality1 = 1;

// Z중 V, YAW 제외
static float R_array_quality2 = 1;

static float R_array_quality3 = 1;

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
static float estimate(float z);

bool velocity_filter_init()
{
    // @todo

    velocity_estimate_filter.P = 0;
    velocity_estimate_filter.predict_P = 0;
    velocity_estimate_filter.estimate_P = 0;

    velocity_estimate_filter.x = 0;
    velocity_estimate_filter.predict_x = 0;
    velocity_estimate_filter.estimate_x = 0;

    velocity_estimate_filter.R = 0.07;
    velocity_estimate_filter.Q = 0.005;
    velocity_estimate_filter.K = 0;
    velocity_estimate_filter.H = 1;

    velocity_estimate_filter.S_inv = 0;

    velocity_estimate_filter.init_flag = 0;

    return true;
}

void velocity_filter_set_last_update_time(float update_time)
{
    velocity_estimate_filter.last_update_time = update_time;
}

float velocity_filter_get_velocity()
{
    return velocity_estimate_filter.predict_x;
}

void velocity_filter_set_velocity(float velocity)
{
    velocity_estimate_filter.init_flag = 1;
    velocity_estimate_filter.predict_x = velocity;
    velocity_estimate_filter.predict_P = 0;
    velocity_estimate_filter.P = 0;
}

float velocity_filter_predict_state(float accel_x, float updated_time)
{
    if (velocity_estimate_filter.init_flag == 1){
        float dt = updated_time - velocity_estimate_filter.last_update_time;
        velocity_estimate_filter.last_update_time = updated_time;

        float A = 1;
        float predict_velocity = velocity_estimate_filter.predict_x + accel_x * dt;

        velocity_estimate_filter.predict_x = predict_velocity;

        // 오차 공분산 계산
        velocity_estimate_filter.predict_P = velocity_estimate_filter.predict_P + velocity_estimate_filter.Q;
    }

    return velocity_estimate_filter.predict_x;
}

bool velocity_filter_estimate_state(float gps_velocity)
{
    // z format = [gps v, yaw rate, gps slip+yaw, gps x, gps y]
    float sigma = 1; // 값의 유효성 기준, 크면 클 수록 통과하기 좋음
    float innovation = 0;
    bool result = false;

    if (velocity_estimate_filter.init_flag != 1) {
        return false;
    }

    // 측정 값과 예측 값 차이
    innovation = gps_velocity - velocity_estimate_filter.predict_x;

    if (velocity_filter_valid_gate(innovation, velocity_estimate_filter.H, velocity_estimate_filter.R, sigma)) {
        // chi square 기준치 통과
        result = true;
    } else {
        // chi square 기준치 미달
        result = false;
    }
    estimate(gps_velocity);
    return result;
}

float estimate(float z)
{
    velocity_estimate_filter.K = velocity_estimate_filter.predict_P * velocity_estimate_filter.S_inv;

    velocity_estimate_filter.estimate_x = velocity_estimate_filter.predict_x +
                                            velocity_estimate_filter.K * (z - velocity_estimate_filter.predict_x);
    velocity_estimate_filter.estimate_P = velocity_estimate_filter.predict_P - velocity_estimate_filter.K * velocity_estimate_filter.predict_P;

    // velocity_estimate_filter.R = C_hat / velocity_estimate_filter.v_estimate_buf.size() + velocity_estimate_filter.estimate_P;
    velocity_estimate_filter.x = velocity_estimate_filter.estimate_x;
    velocity_estimate_filter.predict_x = velocity_estimate_filter.estimate_x;
    velocity_estimate_filter.P = velocity_estimate_filter.estimate_P;
    velocity_estimate_filter.predict_P = velocity_estimate_filter.estimate_P;

    return velocity_estimate_filter.x;
}

bool velocity_filter_valid_gate(float innovation, float H, float R, float sigma)
{
    // innovation 공분산 계산 후
    velocity_estimate_filter.S_inv = 1 / (velocity_estimate_filter.predict_P + R);
    float temp = velocity_estimate_filter.S_inv;

    // Chi-Square Statistic 계산
    float V = innovation * velocity_estimate_filter.S_inv * innovation;
    return V <= (sigma * sigma);
}
