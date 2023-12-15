#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H

#include <vector>
#include "model_matrix.h"
// #include "resource.h"

#define M_PI    3.14159265358979323846
#define M_PI_2  1.57079632679489661923

/**
 * @brief 경로 트레킹을 위한 업데이트 함수 결과
 * @details 주행 중, 목표 도착 등 업데이트 함수 결과 값
 */
typedef enum path_tracking_update_result_
{
    RUNNING = 0x00,
    GOAL,
    OUT_OF_RANGE,
    INVAILED_TIME,
    NOT_READY,
} path_tracking_update_result_t;

/**
 * @brief 경로 위치 정보
 * @details 경로의 각 점들에 대한 목표 및 상태 값들을 저장
 */
typedef struct Point_
{
    double x;
    double y;
    double yaw;
    double k;
    double speed;
} Point;

/**
 * @brief 현재 차량의 상태
 * @details 현재 차량의 상태를 저장
 */
typedef struct ControlState_
{
    double x;
    double y;
    double yaw;
    double steer;
    double v;

    ControlState_()
    {

    }

    ControlState_(double x, double y, double yaw, double steer, double v)
    {
        this->x = x;
        this->y = y;
        this->yaw = yaw;
        this->steer = steer;
        this->v = v;
    }
} ControlState;

typedef struct pid_gain_{
    double kp;
    double ki;
    double kd;
} pid_gain_t;

#define PATH_TRACKER_PID_TYPE_DISTANCE 0
#define PATH_TRACKER_PID_TYPE_STEER 1

#define PATH_TRACKER_LQR_TYPE_Q 0
#define PATH_TRACKER_LQR_TYPE_R 1

class path_tracking_controller
{
public:
    path_tracking_controller();
    path_tracking_controller(const double max_steer_angle, const double max_speed, const double wheel_base);
    ~path_tracking_controller();
    void init(const double max_steer_angle, const double max_speed, const double wheel_base);
    /**
     * @brief 경로 추종을 위한 목표 상태값 및 예측 위치 계산
     * @param [in] time 현재 시간
     */
    path_tracking_update_result_t update(double time);
    /**
     * @brief 경로 저장
     * @param [in] init_state 현재 위치
     * @param [in] points 추가할 맵 데이터
     */
    void set_course(ControlState init_state, std::vector<Point> points);
    /**
     * @brief 경로 추가
     * @param [in] init_state 현재 위치
     * @param [in] points 추가할 맵 데이터
     */
    void add_course(ControlState init_state, std::vector<Point> points);
    /**
     * @brief 목표 지점 까지의 각도 계산
     * @details 주행 시작 시 경로로 이동하기 위한 조향각 계산을 위해 사용
     * @param [in] point 목표 위치
     * @param [out] steer 계산된 조향각
     * @return 주행 가능 여부
     */
    bool get_target_steer_at(Point point, double* steer);
    /**
     * @brief 현재 위치와 경로 사이 거리 계산
     * @param [in] point 현재 위치
     * @param [in] line_point1 경로 뒷점 위치
     * @param [in] line_point2 경로 앞점 위치
     * @return 경로와의 거리(m)
     */
    double distance_between_point_and_line(Point point, Point line_point1, Point line_point2);
    /**
     * @brief 각도 범위 지정
     * @details 입력된 각도 값을 -180º ~ 180º 로 변경
     * @param [in] angle 가공할 각도 값
     * @return 가공한 각도 값
     */
    double pi_2_pi(double angle);

protected:
    /**
     * @brief 현재 이동 가능한 가장 가까운 포인트 인덱스를 계산
     * @param [in] state 현재 상태(위치, 속도, 조향각)
     * @param [in] points 맵 데이터
     * @param [in] pind 가장 최근 타겟 포인트 인덱스
     * @return 타겟 포인트 인덱스
     */
    int calculate_target_index(ControlState state, std::vector<Point> points, int pind);
    /**
     * @brief 타켓 포인트들의 yaw 값 범위 설정(-180 ~ 180 사이로 변경)
     * @param [in] points 맵 데이터
     */
    void smooth_yaw(std::vector<Point> &points);
    /**
     * @brief 다음 스텝 위치값 계산
     * @param [in] state 현재 상태(위치, 속도, 조향각)
     * @param [in] a 목표 가속도
     * @param [in] delta 목표 조향각
     * @param [in] dt 스텝 시간
     */
    ControlState update_state(ControlState state, double a, double delta, double dt);
    /**
     * @brief 다음 스텝 위치값 계산
     * @param [in] state 현재 상태(위치, 속도, 조향각)
     * @param [in] a 목표 가속도
     * @param [in] delta 목표 조향각
     * @param [in] dt 스텝 시간
     */
    ControlState update_state_for_predict(ControlState state, double dt);
    /**
     * @brief 목표 지점으로 이동 가능한지 확인
     * @param [in] dx 현재 위치와 목표 위치 x 값 차이
     * @param [in] dy 현재 위치와 목표 위치 y 값 차이
     * @param [in] yaw 현재 yaw
     * @param [in] steer 현재 조향각
     * @param [in] range_angle 좌우 최대 범위 각도
     * @return 이동 가능 여부
     */
    bool is_point_in_correct_range(double dx, double dy, double yaw, double steer, double range_angle);
    /**
     * @brief 현재 조향각과 목표 조향각 오차에 따른 속도 조정
     * @param [in] state 현재 상태(위치, 속도, 조향각)
     * @param [in] target_velocity 목표 속도
     * @param [in] target_steer 목표 조향각
     * @return 조정된 속도
     */
    double velocity_control_depend_on_steer_error(ControlState state, double target_velocity, double target_steer);
    /**
     * @brief 목표 지점까지 이동할 수 있는 조향각 값 계산
     * @param [in] state 현재 상태(위치, 속도, 조향각)
     * @param [in] target_point 목표 위치
     * @return steer 목표 조향각
     */
    virtual double steering_control(ControlState state, Point target_point);
    /**
     * @brief 목표 지점까지 이동할 수 있는 가속도 값 계산
     * @param [in] state 현재 상태(위치, 속도, 조향각)
     * @param [in] target_point 목표 위치
     * @return accel 목표 가속도 값
     */
    virtual double velocity_control(ControlState state, Point target_point);

protected:
    double dt;                  /** 업데이트 시간 */
    ControlState init_state;    /** 초기 상태 */
    ControlState goal_state;    /** 끝점 상태 */
    std::vector<Point> points;  /** 맵 데이터 */
    ControlState state;         /** 현재 상태 */
    ControlState predict_state; /** 예측 상태 */
    int target_ind;             /** 현재 목표 맵 위치 인덱스 */

    uint32_t updated_tick;      /** 업데이트된 시간(freertos tick) */

    double max_steer_angle;     /** 최대 동작 가능한 조향 각도 */
    double max_speed;           /** 최대 동작 가능한 차량 속도 */
    double wheel_base;          /** 차량 앞 바퀴와 뒷 바퀴 사이 거리 */

    double distance_error;      /** 경로와의 거리 오차 */
    double yaw_error;           /** 경로와의 각도 오차 */
    double steer_error;         /** 실 조향 각도와 목표 조향 각도 오차 */

    double target_steer;        /** 목표 조향 각 */
    double target_velocity;     /** 목표 속도 */

    int jumping_point;          /** 앞점 추가 인덱스 */

public:
    /**
     * @brief 현재 상태 반환
     * @return 가장 최근에 갱신된 현재 상태
     */
    ControlState get_state() const {
        return this->state;
    }
    /**
     * @brief 예측 상태 반환
     * @return 예측된 상태
     */
    ControlState get_predict_state() const {
        return this->predict_state;
    }
    /**
     * @brief 현재 상태 갱신
     * @param state 위치 및 정보(속도, 조향)
     */
    void set_state(ControlState state) {
        this->state = state;
        this->predict_state = state;
    }
    /**
     * @brief 현재 상태 갱신
     * @param state 위치 및 정보(속도, 조향)
     * @param time 갱신 시간(freertos tick)
     */
    void set_state(ControlState state, uint32_t time) {
        this->state = state;
        this->updated_tick = time;
        this->predict_state = state;
    }
    /**
     * @brief 현재 조향 각도 갱신
     * @param steer 변경할 조향 각도
     */
    void set_steer(double steer) {
        this->state.steer = steer;
        this->predict_state.steer = steer;
    }
    /**
     * @brief 현재 yaw 갱신
     * @param steer 변경할 yaw
     */
    void set_yaw(double yaw) {
        this->state.yaw = yaw;
        this->predict_state.yaw = yaw;
    }
    /**
     * @brief 현재 목표 포인트 인덱스 변경
     * @param target_index 변경할 목표 인덱스
     */
    void set_target_index(int target_index) {
        this->target_ind = target_index;
    }
    /**
     * @brief 맵 데이터 반환
     * @return 맵 데이터
     */
    std::vector<Point> get_points() const {
        return this->points;
    }
    /**
     * @brief 현재 목표 포인트 인덱스 반환
     * @return 현재 목표 포인트 인덱스
     */
    int get_target_index() const {
        return this->target_ind;
    }
    /**
     * @brief 현재 목표점에서 예측을 위한 앞점 인덱스 반환
     * @return 현재 목표점에서 예측을 위한 앞점 인덱스
     */
    int get_jumped_target_index() const {
        if (this->target_ind + this->jumping_point >= this->points.size()) {
            return this->points.size() - 1;
        } else {
            return this->target_ind + this->jumping_point;
        }
    }
    /**
     * @brief 맵 데이터 제거
     * @param num 앞점부터 제거할 포인트 개수
     */
    void remove_points(size_t num) {
        if (num == 0) {
            this->points.clear();
        } else if (this->points.size() > num) {
            this->points.erase(begin(this->points), begin(this->points) + num);
        }
    }
    /**
     * @brief 현재 가지고 있는 맵 데이터 개수 반환
     * @return 현재 가지고 있는 맵 데이터 개수
     */
    size_t get_remain_point() const {
        return this->points.size() - target_ind - 1;
    }
    /**
     * @brief 현재 경로와의 거리 오차 반환
     * @return 현재 경로와의 거리 오차
     */
    double get_distance_error() {
        return this->distance_error;
    }
    /**
     * @brief 현재 경로와의 각도 오차 반환
     * @return 현재 경로와의 각도 오차
     */
    double get_yaw_error() {
        return this->yaw_error;
    }
    /**
     * @brief 현재 경로와의 조향 각도 오차 반환
     * @return 현재 경로와의 조향 각도 오차
     */
    double get_steer_error() {
        return this->steer_error;
    }
    /**
     * @brief 현재 목표 조향 각도 반환
     * @return 현재 목표 조향 각도
     */
    double get_target_steer() {
        return this->target_steer;
    }
    /**
     * @brief 현재 목표 속도 반환
     * @return 현재 목표 속도
     */
    double get_target_velocity() {
        return this->target_velocity;
    }

    virtual void get_gain(int gain_index, double* gain_value);
    virtual void set_gain(int gain_index, double* gain_value);
};

#endif
