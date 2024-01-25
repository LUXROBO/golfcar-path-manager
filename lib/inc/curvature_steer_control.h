#pragma once

// std
#include <vector>
#include <fstream>


#include "lqr_pid_control.h"
#include "path_manager.h"


class curvature_steer_control : public path_tracker
{
public:
    curvature_steer_control();
    curvature_steer_control(const double max_steer_angle, const double max_speed, const double wheel_base);
    ~curvature_steer_control();

public:
    virtual void set_gain(int gain_index, double* gain_value);
    virtual void get_gain(int gain_index, double* gain_value);

private:
    virtual double steering_control(pt_control_state_t state, path_point_t target_point);
    virtual double velocity_control(pt_control_state_t state, path_point_t target_point);
    /**
     * @brief point1을 지나는 yaw각도를 가진 직선위에 있는 점 중 point2와의 거리가 point1와의 거리와 일치하는 점을 탐색
     * @param [in] points1 직선위의 점
     * @param [in] points2 거리가 동일해야하는 점
     * @param [in] points2 point1을 지나는 직선의 기울기
     * @return point1, point2와 거리가 동일한 점(곡률 적용)
     * @TODO 직선일 경우 구분해야함
     */
    path_point_t get_path_circle(path_point_t point1, path_point_t point2, double slope);





private:

public:
};
