#pragma once

// std
#include <stddef.h>
#include <stdint.h>
#include <vector>

#include "path_manager_def.h"

class path_tracker
{
public:
    path_tracker();
    path_tracker(const float max_steer_angle, const float max_speed, const float wheel_base, const float center_to_gps_distance);
    ~path_tracker();

public:
    virtual void get_gain(pt_gain_type_t gain_index, float* gain_value) = 0;
    virtual void set_gain(pt_gain_type_t gain_index, float* gain_value) = 0;

protected:
    /**
     * @brief 목표점까지 이동할 수 있는 조향각 계산
     * @param [in] state 현재 상태
     * @param [in] target 목표점
     * @return 목표 조향각[rad]
     */
    virtual float steering_control(pt_control_state_t state, std::vector<path_point_t> target) = 0;

    /**
     * @brief 목표점까지 이동할 수 있는 주행 속도 계산
     * @param [in] state 현재 상태
     * @param [in] target 목표점
     * @return 목표 주행 속도[m/s]
     */
    virtual float velocity_control(pt_control_state_t state, path_point_t target) = 0;

public:
    /**
     * @brief path_tracker 초기화
     * @param [in] max_steer_angle 최대 조향 각[rad]
     * @param [in] max_speed 최대 속도[m/s]
     * @param [in] wheel_base 앞뒤 바퀴 간격[m]
     * @param [in] center_to_gps_distance 차량 중심과 gps 사이 거리, gps가 차량 앞에 있다면 +, 뒤에 있다면 -[m]
     */
    void init(const float max_steer_angle, const float max_speed, const float wheel_base, const float center_to_gps_distance);

    /**
     * @brief 경로 설정
     * @param [in] init_state 초기 상태
     * @param [in] points 경로
     */
    void set_path_points(pt_control_state_t init_state, std::vector<path_point_t> points);

    /**
     * @brief 경로 추적을 위한 목표 상태 및 예측 위치 계산
     * @param [in] time 함수 호출 시간[s]
     */
    pt_update_result_t update(float dt);

    /**
     * @brief 목표점으로 이동 가능한지 확인
     * @param [in] current 현재 상태
     * @param [in] target 목표점
     * @param [in] range 좌우 최대 범위[rad]
     * @return 이동 가능 여부
     */
    bool is_moveable_point(pt_control_state_t current, path_point_t target, float range);

    /**
     * @brief 현재 상태에서 목표점으로 이동하기 위한 조향각 반환
     * @param [in] current 현재 상태
     * @param [in] target 목표점
     * @param [out] steer 계산된 조향각[rad]
     * @return 이동 가능 여부
     */
    bool get_steer_at_moveable_point(pt_control_state_t current, path_point_t target, float* steer);

public:
    /**
     * @brief 예측 상태 반환
     * @return 예측 상태
     */
    pt_control_state_t get_state() const {
        return this->state;
    }

    /**
     * @brief 현재 상태 갱신
     * @param [in] state 현재 상태
     */
    void set_state(pt_control_state_t state) {
        this->state = state;
    }

    /**
     * @brief 현재 상태 갱신
     * @param [in] state 현재 상태
     * @param [in] time 함수 호출 시간[s]
     */
    void set_state(pt_control_state_t state, float time) {
        this->updated_time = time;
        this->set_state(state);
    }

    /**
     * @brief 현재 위치 갱신
     * @details 위치 갱신 주기가 경로 추적 갱신 주기보다 빠를 경우 사용
     * @param [in] x 변경할 위치 X 좌표[m]
     * @param [in] y 변경할 위치 Y 좌표[m]
     */
    void set_position(double x, double y) {
        this->state.x = x;
        this->state.y = y;
    }

    /**
     * @brief 현재 Yaw 갱신
     * @details Yaw 갱신 주기가 경로 추적 갱신 주기보다 빠를 경우 사용
     * @param [in] yaw 변경할 Yaw[rad]
     */
    void set_yaw(float yaw) {
        this->state.yaw = yaw;
    }

    /**
     * @brief 현재 조향각 갱신
     * @details 조향각 갱신 주기가 경로 추적 갱신 주기보다 빠를 경우 사용
     * @param [in] steer 갱신할 조향각[rad]
     */
    void set_steer(float steer) {
        this->state.steer = steer;
    }

    /**
     * @brief 현재 주행 속도 갱신
     * @details 주행 속도 갱신 주기가 경로 추적 갱신 주기보다 빠를 경우 사용
     * @param [in] velocity 갱신할 주행 속도[m/s]
     */
    void set_velocity(float velocity) {
        this->state.v = velocity;
    }

    /**
     * @brief 현재 목표 포인트 인덱스 반환
     * @return 현재 목표 포인트 인덱스
     */
    int get_target_point_index() const {
        return this->target_point_index;
    }

    /**
     * @brief 현재 목표점에서 예측을 위한 앞점 인덱스 반환
     * @return 현재 목표점에서 예측을 위한 앞점 인덱스
     */
    int get_front_target_point_index() const {
        unsigned int index = this->target_point_index + this->target_index_offset;

        if (index >= this->points.size()) {
            index = this->points.size() - 1;
        }

        return index;
    }

    /**
     * @brief 현재 목표점에서 예측을 위한 앞점 인덱스 반환
     * @return 현재 목표점에서 예측을 위한 앞점 인덱스
     */
    int get_front_target_point_index(int index2) const {
        unsigned int index = index2 + this->target_index_offset;

        if (index >= this->points.size()) {
            index = this->points.size() - 1;
        }

        return index;
    }

    /**
     * @brief 남은 경로점 개수 반환
     * @return 남은 경로점 개수
     */
    size_t get_remain_point_num() const {
        return this->points.size() - this->target_point_index - 1;
    }

    /**
     * @brief 현재 경로의 거리 오차 반환
     * @return 거리 오차[m]
     */
    float get_distance_error() {
        return this->distance_error;
    }

    /**
     * @brief 현재 경로의 방향 오차 반환
     * @return 방향 오차[rad]
     */
    float get_yaw_error() {
        return this->yaw_error;
    }

    /**
     * @brief 목표 조향각 반환
     * @return 목표 조향각[rad]
     */
    float get_target_steer() {
        return this->target_steer;
    }

    /**
     * @brief 목표 주행 속도 반환
     * @return 목표 주행 속도[m/s]
     */
    float get_target_velocity() {
        return this->target_velocity;
    }
    pt_control_state_t update_predict_state2(pt_control_state_t state, float dt);

    /**
     * @brief 목표 주행 속도 반환
     * @return 목표 주행 속도[m/s]
     */
    float get_revise_target_steer() {
        return this->revise_target_steer;
    }

private:
    /**
     * @brief 예측 상태 계산
     * @param [in] state 현재 상태
     * @param [in] dt 스텝 시간[s]
     */
    pt_control_state_t update_predict_state(pt_control_state_t state, float dt);

    path_point_t get_point_cross_two_line(path_point_t point1, float slope1, path_point_t point2, float slope2);

    /**
     * @brief 목표점 인덱스를 계산
     * @param [in] state 현재 상태
     * @param [in] points 경로
     * @param [in] start_index 검색 시작 경로점 인덱스
     * @return 목표점 인덱스
     */
    int calculate_target_index(pt_control_state_t state, std::vector<path_point_t>& points, int start_index);

    /**
     * @brief 현재 조향각과 목표 조향각 오차에 따른 속도 조정
     * @param [in] state 현재 상태
     * @param [in] target_velocity 목표 주행 속도[m/s]
     * @param [in] target_steer 목표 조향각[rad]
     * @return 조정된 속도[m/s]
     */
    float velocity_control_depend_on_steer_error(pt_control_state_t state, float target_velocity, float target_steer);

protected:
    float max_steer_angle;              /** 최대 조향 각도[rad] */
    float max_speed;                    /** 최대 주행 속도[m/s] */
    float wheel_base;                   /** 차량 앞뒤 바퀴 간격[m] */

    float dt;                                  /** 업데이트 스텝 시간[s] */
    float updated_time;                        /** 업데이트된 시간[s] */

    pt_control_state_t init_state;              /** 초기 상태 */
    pt_control_state_t goal_state;              /** 끝점 상태 */
    pt_control_state_t state;           /** 예측 상태 */

    std::vector<path_point_t> points;           /** 경로 데이터 */

    float distance_error;                      /** 경로와의 거리 오차[m] */
    float yaw_error;                           /** 경로와의 방향 오차[rad] */

    float target_steer;                        /** 목표 조향 각[rad] */
    float target_velocity;                     /** 목표 속도[m/s] */
    float revise_target_steer;                        /** 시뮬레이션용 목표 조향 각[rad] */

    unsigned int target_point_index;                     /** 현재 목표 맵 위치 인덱스 */
    int target_index_offset;                    /** 앞점 추가 인덱스 */
    int max_look_ahead_num;

    float g_vl;
    float g_vr;

    float lf;
    float lr;

public:
    /**
     * @brief 현재 위치와 경로점 거리 계산
     * @param [in] current_point 현재 위치
     * @param [in] point 경로점
     * @return 경로점과의 거리[m]
     */
    static float get_point_distance(path_point_t current_point, path_point_t point);

    /**
     * @brief 현재 위치와 경로 사이 거리 계산
     * @param [in] current_point 현재 위치
     * @param [in] line_point1 경로 뒷점 위치
     * @param [in] line_point2 경로 앞점 위치
     * @return 경로와의 거리[m] (-: 왼쪽에 위치 함, +: 오른쪽에 위치 함)
     */
    static float get_line_distance(path_point_t current_point, path_point_t line_point1, path_point_t line_point2);

    /**
     * @brief 경로점들의 yaw 값 범위를 -π ~ π로 변경
     * @param [in] points 경로
     */
    static void smooth_yaw(std::vector<path_point_t> &points);

    /**
     * @brief 각도 범위 지정
     * @details 입력된 각도 값을 -π ~ π 로 변경
     * @param [in] angle 가공할 각도 값[rad]
     * @return 가공된 각도 값[rad]
     */
    static float pi_to_pi(float angle);

    /**
     * @brief point1을 지나는 yaw각도를 가진 직선위에 있는 점 중 point2와의 거리가 point1와의 거리와 일치하는 점을 탐색
     * @param [in] points1 직선위의 점
     * @param [in] points2 거리가 동일해야하는 점
     * @param [in] slope point1을 지나는 직선의 기울기
     * @return point1, point2와 거리가 동일한 점(곡률 적용)
     * @TODO 직선일 경우 구분해야함
     */
    static path_point_t get_path_circle(path_point_t point1, path_point_t point2, float slope);
};
