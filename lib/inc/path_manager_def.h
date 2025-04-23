#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// std
#include <stddef.h>
#include <stdint.h>


#define PT_M_PI    3.14159265358979323846
#define PT_M_PI_2  1.57079632679489661923

/**
 * @brief 경로 트레킹을 위한 업데이트 함수 결과
 * @details 주행 중, 목표 도착 등 업데이트 함수 결과 값
 */
typedef enum path_tracker_update_result
{
    PT_UPDATE_RESULT_RUNNING = 0x0000,
    PT_UPDATE_RESULT_GOAL,
    PT_UPDATE_RESULT_NOT_FOUND_TARGET,
    PT_UPDATE_RESULT_INVAILED_TIME,
    PT_UPDATE_RESULT_NOT_READY,
} pt_update_result_t;

/**
 * @brief 경로 위치 정보
 * @details 경로의 각 점들에 대한 목표 및 상태 값들을 저장
 */
#pragma pack(push, 1)
typedef struct path_point
{
    double x;
    double y;
    float yaw;
    float k;
    float speed;
} path_point_t;
#pragma pack(pop)

/**
 * @brief 현재 차량의 상태
 * @details 현재 차량의 상태를 저장
 */
#pragma pack(push, 1)
typedef struct path_tracker_control_state
{
    double x;
    double y;
    float yaw;
    float steer;
    float v;
} pt_control_state_t;
#pragma pack(pop)

#define PT_GAIN_TYPE_PID_DISTANCE   0
#define PT_GAIN_TYPE_PID_STEER      1
#define PT_GAIN_TYPE_LQR_Q          0
#define PT_GAIN_TYPE_LQR_R          1
typedef int pt_gain_type_t;

#ifdef __cplusplus
}
#endif
