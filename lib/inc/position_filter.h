#pragma once

#include <path_manager.h>
#include <qformat.h>
#include <model_matrix.h>
#include <vector>

// std
#include <stddef.h>
#include <stdint.h>

#define POSITION_FILTER_INIT_YAW 1
#define POSITION_FILTER_INIT_XY 2
#define POSITION_FILTER_INIT_BOTH 3

#define POSITION_FILTER_QUALITY_ALL                     0 /**< 모든 데이터 사용*/
#define POSITION_FILTER_QUALITY_EXCEPT_YAW              1 /**< GPS로 계산한 YAW만 사용하지 않음*/
#define POSITION_FILTER_QUALITY_EXCEPT_GPS_DERIVATIVE   2 /**< GPS로 계산한 속도와 YAW를 사용하지 않음*/
#define POSITION_FILTER_QUALITY_ONLY_IMU                3 /**< IMU만 사용(GPS 데이터 모두 무시)*/

typedef struct position_filter_z_format_
{
    float gps_v;
    float yaw_rate;
    float gps_yaw;
    double gps_x;
    double gps_y;
} position_filter_z_format_t;

typedef enum position_filter_init_state_
{
    position_filter_init_yaw = 1,
    position_filter_init_xy,
    position_filter_init_both
} position_filter_init_state_t;

/**
 * @brief position filter 초기화
 * @return true 초기화에 성공한 경우
 * @return false 초기화에 실패한 경우
 */
bool position_filter_init();

/**
 * @brief 필터 상태 초기화 여부 확인
 * @return true 필터 상태가 초기화 된 경우
 * @return false 필터 상태가 초기화 되지 않은 경우
 */
bool position_filter_is_init();

/**
 * @brief 필터 상태 초기화 여부 확인
 * @return true 필터 상태가 초기화 된 경우
 * @return false 필터 상태가 초기화 되지 않은 경우
 */
position_filter_init_state_t position_filter_get_init_state();

/**
 * @brief 필터의 예측 값을 초기화
 * @param [in] position 차량의 현재 상태
 */
void position_filter_set_position(pt_control_state_t position);

/**
 * @brief 필터의 예측 x, y값을 초기화
 * @param [in] x 차량의 현재 x축 위치
 * @param [in] y 차량의 현재 y축 위치
 */
void position_filter_set_xy(double x, double y);

/**
 * @brief 필터의 마지막 업데이트 시간을 수정
 * @param [in] update_time 필터가 마지막으로 업데이트 된 시간 [s]
 */
void position_filter_set_last_update_time(float update_time);

/**
 * @brief 필터의 예측 yaw 값을 초기화
 * @param [in] yaw 차량의 현재 yaw 각도
 */
void position_filter_set_yaw(float yaw);

void position_filter_set_R(int gps_quality);


/**
 * @brief 필터의 예측 값을 리턴
 * @return pt_control_state_t 필터의 예측된 차량의 상태 값
 */
pt_control_state_t position_filter_get_state();

/**
 * @brief 필터의 예측 값을 리턴
 * @return pt_control_state_t 필터의 예측된 차량의 상태 값
 */
ModelMatrix position_filter_get_predict_x();

float position_filter_get_chi_square_value();

void position_filter_set_predict_x(ModelMatrix new_x);

/**
 * @brief 차량의 주행 파라미터로 차량의 상태를 예측
 * @param [in] v 차량의 현재 속도
 * @param [in] steer 차량의 현재 조향 각도
 * @param [in] updated_time 현재 업데이트 된 시간 시간[s]
 * @return pt_control_state_t 예측된 x,y 위치를 path manager에서 관리하는 형태로 리턴
 */
pt_control_state_t position_filter_predict_state(float v, float steer, float updated_time);

/**
 * @brief GPS의 위치 값으로 실 차량의 위치를 추종
 * @param [in] z_value 측정 센터 값 @ref position_filter_z_format_t
 * @param [in] quality position filter의 quality @ref POSITION_FILTER_QUALITY
 * @return bool 유효한 추정을 진행했는지에 대한 여부
 */
bool position_filter_estimate_state(position_filter_z_format_t z_value, int quality);

/**
 * @brief 필터의 측정 값의 유효성을 검사
 * @param [in] innovation 측정 값과 예측 값의 차이
 * @param [in] H 측정값 도출 행렬
 * @param [in] sigma 유효성 기준치
 * @return pt_control_state_t 추종된 차량의 위치
 */
bool position_filter_valid_gate(ModelMatrix innovation, ModelMatrix H, ModelMatrix R, float sigma);
