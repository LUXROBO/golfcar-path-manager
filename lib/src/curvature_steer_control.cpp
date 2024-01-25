#include "curvature_steer_control.h"

#include <stdio.h>
#include <iostream>
#include <cmath>


curvature_steer_control::curvature_steer_control()
{

}

curvature_steer_control::curvature_steer_control(const double max_steer_angle, const double max_speed, const double wheel_base)
: path_tracker(max_steer_angle, max_speed, wheel_base)
{

}

curvature_steer_control::~curvature_steer_control()
{

}

void curvature_steer_control::set_gain(int gain_index, double* gain_value)
{

}

void curvature_steer_control::get_gain(int gain_index, double* gain_value)
{

}

double curvature_steer_control::steering_control(pt_control_state_t state, path_point_t target_point)
{
    path_point_t current_state_to_point = path_point_t{this->predict_state.x, this->predict_state.y, 0, 0, 0};
    path_point_t circle1 = get_path_circle(current_state_to_point, target_point, math.tan(path_tracker::pi_to_pi(this->predict_state.yaw + PT_M_PI_2)));
    path_point_t circle2 = get_path_circle(target_point, current_state_to_point, math.tan(path_tracker::pi_to_pi(target_point.yaw + PT_M_PI_2)));

    double r1 = 1 / circle1.k;
    double r2 = 1 / circle2.k;  // 지금은 r2가 더 큰 원이라 생각 나중에 수정
    double circle_to_circle = math.sqrt(math.pow(r1.x - r2.x, 2) + math.pow(r1.y - r2.y, 2));

    double rr1 = r1 - circle_to_circle;
    // double r22 = 2 * r1 - rr1 - r2;
    double r22 = r1 + circle_to_circle - r2;

    double new_point_to_r1 = r1 - rr2 / 2;
    double new_point_to_r1_angle = math.atan2(r1.y - r2.y), (r1.x - r2.x);
    path_point_t new_circle_point = {r.x + new_point_to_r1 * math.cos(new_point_to_r1_angle),
                                     r.y + new_point_to_r1 * math.sin(new_point_to_r1_angle), 0, 0, 0};
    
    path_point_t circle3 = get_path_circle(new_circle_point, target_point, math.tan(new_point_to_r1_angle));

    return steer;
}

double curvature_steer_control::velocity_control(pt_control_state_t state, path_point_t target_point)
{
    return target_point.speed;
}

path_point_t curvature_steer_control::get_path_circle(path_point_t point1, path_point_t point2, double slope)
{
    // double orthogonal_yaw = path_tracker::pi_to_pi(yaw + PT_M_PI_2);
    double a = slope;
    double b = point1.y - a * point1.x;
    double c = point1.y - b;
    double d = point2.y - b;

    double x = (math.pow(point1.x,2) + math.pow(c,2) - math.pow(point2.x,2) - math.pow(d,2)) / (-2 * (point1.x + (a * c) - point2.x - (a * d)));
    double y = a * x + b;
    double distance = math.sqrt(math.pow(x - point1.x, 2) + math.pow(y - point1.y, 2))

    return path_point_t{x, y, 0, 1 / distance, 0};
}

