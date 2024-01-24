#pragma once

// std
#include <stddef.h>
#include <stdint.h>
#include <vector>


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
typedef struct path_point
{
    double x;
    double y;
    double yaw;
    double k;
    double speed;
} path_point_t;

/**
 * @brief 현재 차량의 상태
 * @details 현재 차량의 상태를 저장
 */
typedef struct path_tracker_control_state
{
    double x;
    double y;
    double yaw;
    double steer;
    double v;
} pt_control_state_t;

#define PT_GAIN_TYPE_PID_DISTANCE   0
#define PT_GAIN_TYPE_PID_STEER      1
#define PT_GAIN_TYPE_LQR_Q          0
#define PT_GAIN_TYPE_LQR_R          1
typedef int pt_gain_type_t;

class path_tracker
{
public:
    path_tracker();
    path_tracker(const double max_steer_angle, const double max_speed, const double wheel_base);
    ~path_tracker();

public:
    virtual void get_gain(pt_gain_type_t gain_index, double* gain_value) = 0;
    virtual void set_gain(pt_gain_type_t gain_index, double* gain_value) = 0;

protected:
    /**
     * @brief 목표점까지 이동할 수 있는 조향각 계산
     * @param [in] state 현재 상태
     * @param [in] target 목표점
     * @return 목표 조향각[rad]
     */
    virtual double steering_control(pt_control_state_t state, path_point_t target) = 0;

    /**
     * @brief 목표점까지 이동할 수 있는 주행 속도 계산
     * @param [in] state 현재 상태
     * @param [in] target 목표점
     * @return 목표 주행 속도[m/s]
     */
    virtual double velocity_control(pt_control_state_t state, path_point_t target) = 0;

public:
    /**
     * @brief path_tracker 초기화
     * @param [in] max_steer_angle 최대 조향 각[rad]
     * @param [in] max_speed 최대 속도[m/s]
     * @param [in] wheel_base 앞뒤 바퀴 간격[m]
     */
    void init(const double max_steer_angle, const double max_speed, const double wheel_base);

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
    pt_update_result_t update(double time);

    /**
     * @brief 목표점으로 이동 가능한지 확인
     * @param [in] current 현재 상태
     * @param [in] target 목표점
     * @param [in] range 좌우 최대 범위[rad]
     * @return 이동 가능 여부
     */
    bool is_moveable_point(pt_control_state_t current, path_point_t target, double range);

    /**
     * @brief 현재 상태에서 목표점으로 이동하기 위한 조향각 반환
     * @param [in] current 현재 상태
     * @param [in] target 목표점
     * @param [out] steer 계산된 조향각[rad]
     * @return 이동 가능 여부
     */
    bool get_steer_at_moveable_point(pt_control_state_t current, path_point_t target, double* steer);

public:
    /**
     * @brief 현재 상태 반환
     * @return 가장 최근에 갱신된 현재 상태
     */
    pt_control_state_t get_last_updated_state() const {
        return this->last_updated_state;
    }

    /**
     * @brief 예측 상태 반환
     * @return 예측 상태
     */
    pt_control_state_t get_predict_state() const {
        return this->predict_state;
    }

    /**
     * @brief 현재 상태 갱신
     * @param [in] state 현재 상태
     */
    void set_state(pt_control_state_t state) {
        this->predict_state = state;
        this->last_updated_state = state;
    }

    /**
     * @brief 현재 상태 갱신
     * @param [in] state 현재 상태
     * @param [in] time 함수 호출 시간[s]
     */
    void set_state(pt_control_state_t state, double time) {
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
        this->last_updated_state.x = x;
        this->predict_state.x = x;

        this->last_updated_state.y = y;
        this->predict_state.y = y;
    }

    /**
     * @brief 현재 Yaw 갱신
     * @details Yaw 갱신 주기가 경로 추적 갱신 주기보다 빠를 경우 사용
     * @param [in] yaw 변경할 Yaw[rad]
     */
    void set_yaw(double yaw) {
        this->last_updated_state.yaw = yaw;
        this->predict_state.yaw = yaw;
    }

    /**
     * @brief 현재 조향각 갱신
     * @details 조향각 갱신 주기가 경로 추적 갱신 주기보다 빠를 경우 사용
     * @param [in] steer 갱신할 조향각[rad]
     */
    void set_steer(double steer) {
        this->last_updated_state.steer = steer;
        this->predict_state.steer = steer;
    }

    /**
     * @brief 현재 주행 속도 갱신
     * @details 주행 속도 갱신 주기가 경로 추적 갱신 주기보다 빠를 경우 사용
     * @param [in] velocity 갱신할 주행 속도[m/s]
     */
    void set_velocity(double velocity) {
        this->last_updated_state.v = velocity;
        this->predict_state.v = velocity;
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
        int index = this->target_point_index + this->target_index_offset;

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
    double get_distance_error() {
        return this->distance_error;
    }

    /**
     * @brief 현재 경로의 방향 오차 반환
     * @return 방향 오차[rad]
     */
    double get_yaw_error() {
        return this->yaw_error;
    }

    /**
     * @brief 목표 조향각 반환
     * @return 목표 조향각[rad]
     */
    double get_target_steer() {
        return this->target_steer;
    }

    /**
     * @brief 목표 주행 속도 반환
     * @return 목표 주행 속도[m/s]
     */
    double get_target_velocity() {
        return this->target_velocity;
    }

private:
    /**
     * @brief 예측 상태 계산
     * @param [in] state 현재 상태
     * @param [in] dt 스텝 시간[s]
     */
    pt_control_state_t update_predict_state(pt_control_state_t state, double dt);

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
    double velocity_control_depend_on_steer_error(pt_control_state_t state, double target_velocity, double target_steer);

protected:
    double max_steer_angle;              /** 최대 조향 각도[rad] */
    double max_speed;                    /** 최대 주행 속도[m/s] */
    double wheel_base;                   /** 차량 앞뒤 바퀴 간격[m] */

    double dt;                                  /** 업데이트 스텝 시간[s] */
    double updated_time;                        /** 업데이트된 시간[s] */

    pt_control_state_t init_state;              /** 초기 상태 */
    pt_control_state_t goal_state;              /** 끝점 상태 */
    pt_control_state_t last_updated_state;      /** 현재 상태 */
    pt_control_state_t predict_state;           /** 예측 상태 */

    std::vector<path_point_t> points;           /** 경로 데이터 */

    double distance_error;                      /** 경로와의 거리 오차[m] */
    double yaw_error;                           /** 경로와의 방향 오차[rad] */

    double target_steer;                        /** 목표 조향 각[rad] */
    double target_velocity;                     /** 목표 속도[m/s] */

    int target_point_index;                     /** 현재 목표 맵 위치 인덱스 */
    int target_index_offset;                    /** 앞점 추가 인덱스 */

public:
    /**
     * @brief 현재 위치와 경로점 거리 계산
     * @param [in] current_point 현재 위치
     * @param [in] point 경로점
     * @return 경로점과의 거리[m]
     */
    static double get_point_distance(path_point_t current_point, path_point_t point);

    /**
     * @brief 현재 위치와 경로 사이 거리 계산
     * @param [in] current_point 현재 위치
     * @param [in] line_point1 경로 뒷점 위치
     * @param [in] line_point2 경로 앞점 위치
     * @return 경로와의 거리[m] (-: 왼쪽에 위치 함, +: 오른쪽에 위치 함)
     */
    static double get_line_distance(path_point_t current_point, path_point_t line_point1, path_point_t line_point2);

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
    static double pi_to_pi(double angle);
};
