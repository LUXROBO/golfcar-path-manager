#pragma once

#include <path_manager.h>
#include <qformat.h>
#include <model_matrix_double.h>
#include <vector>

// std
#include <stddef.h>
#include <stdint.h>

/**
 * @brief position filter 초기화
 * @return true 초기화에 성공한 경우
 * @return false 초기화에 실패한 경우
 */
bool position_filter_init();

/**
 * @brief 차량의 주행 파라미터로 x,y 위치를 예측
 * @param [in] v 차량의 현재 속도
 * @param [in] steer 차량의 현재 조향 각도
 * @param [in] yaw 차량의 현재 yaw
 * @param [in] dt 마지막 업데이트 후(predict, estimate 포함) 시간[s]
 * @return pt_control_state_t 예측된 x,y 위치를 path manager에서 관리하는 형태로 리턴
 */
pt_control_state_t position_filter_predict_xy(double v, double steer, double yaw, double dt);

/**
 * @brief GPS의 위치 값으로 실 차량의 위치를 추종
 * @param [in] gps_pos GPS로 수신한 차량의 위치, 구조체의 x, y만 채워도 동작
 * @param [in] quality GPS의 RTK quality
 * @return pt_control_state_t 추종된 차량의 위치
 */
pt_control_state_t position_filter_estimate_xy_with_gps(pt_control_state_t gps_pos, int quality);

/**
 * @brief IMU 센서 값으로 차량의 yaw를 예측
 * @param [in] angular_velocity IMU의 z축 각속도[rad/s]
 * @param [in] dt 마지막 업데이트 후(predict, estimate 포함) 시간[s]
 * @return pt_control_state_t 예측된 차량의 yaw
 */
pt_control_state_t position_filter_predict_yaw(double angular_velocity, double dt);

/**
 * @brief GPS의 위치 변화를 통해 계산된 yaw 값으로 실 차량의 yaw를 추종
 * @param [in] z GPS의 위치 변화를 통해 계산된 yaw
 * @return pt_control_state_t 추종된 차량의 yaw
 */
double position_filter_estimate_yaw(double z);

/**
 * @brief 필터의 위치 값을 갱신
 * @param [in] position 갱신한 위치 값
 */
void position_filter_set_position(pt_control_state_t position);

/**
 * @brief 필터의 예측 위치를 요청
 * @return pt_control_state_t 필터에서 예측된 위치
 */
pt_control_state_t position_filter_get_position();

/**
 * @brief 차량의 x, y 위치를 설정
 * @param [in] x 차량의 위치
 */
void position_filter_set_xy(pt_control_state_t x);

/**
 * @brief 차량의 예측된 x, y 위치를 요청
 * @return pt_control_state_t 차량의 위치
 */
pt_control_state_t position_filter_get_xy();

/**
 * @brief 차의 yaw를 설정
 * @param [in] yaw 차량의 yaw
 */
void position_filter_set_yaw(double yaw);

/**
 * @brief 차량의 예측된 yaw를 요청
 * @return double 차량의 yaw
 */
double position_filter_get_yaw();


void position_filter_set_yaw_R(double yaw_R);

bool position_filter_is_init_xy();
