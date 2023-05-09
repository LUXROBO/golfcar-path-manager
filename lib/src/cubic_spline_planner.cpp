#include "cubic_spline_planner.h"

#include "arm_math.h"
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <iostream>

#define SPLINE_TAG "SPLINE_TAG"

CubicSpline1D::CubicSpline1D(std::vector<float> x, std::vector<float> y)
{
    std::vector<float> diff_x;
    for (uint32_t i = 0; i < x.size() - 1; i++) {
        float diff = x[i + 1] - x[i];
        diff_x.push_back(diff);
        if (diff < 0) {
            return;
            //throw std::runtime_error("x coordinates must be sorted in ascending order");
        }
    }

    this->x = x;
    this->y = y;

    this->a = y;
    ModelMatrix coeff_a = this->calculate_a(diff_x);
    ModelMatrix coeff_b = this->calculate_b(diff_x, this->a);
    float temp_float = 0.001;
    q31_t temp_q31;
    arm_float_to_q31(&temp_float, &temp_q31, 1);
    this->c = coeff_a.inverse(temp_q31) * coeff_b;

    for (uint32_t i = 0; i < this->x.size() - 1; i++) {
        float temp_c0;
        float temp_c1;
        q31_t temp_c0_q31 = this->c.get(i, 0);
        q31_t temp_c1_q31 = this->c.get(i + 1, 0);

        arm_q31_to_float(&temp_c0_q31, &temp_c0, 1);
        arm_q31_to_float(&temp_c1_q31, &temp_c1, 1);

        float d_temp = (temp_c1_q31 - temp_c0_q31) / (3.0 * diff_x[i]);
        float b_temp = 1.0 / diff_x[i] * (this->a[i + 1] - this->a[i]) - diff_x[i] / 3.0 * (2.0 * temp_c0_q31 + temp_c1_q31);
        this->d.push_back(d_temp);
        this->b.push_back(b_temp);
    }
}

CubicSpline1D::~CubicSpline1D()
{

}

float CubicSpline1D::calculate_position(float x)
{
    if (x < this->x[0]) {
        return 0;
    } else if (x > this->x[this->x.size()-1]) {
        return 0;
    }
    int i = this->search_index(x);
    if (i < 0) {
        i = 0;
    }
    float dx = x - this->x[i];
    q31_t temp_c_q31 = this->c.get(i, 0);
    float temp_c;
    arm_q31_to_float(&temp_c_q31, &temp_c, 1);
    float position = this->a[i] + this->b[i] * dx + temp_c * std::pow(dx, 2.0) + this->d[i] * std::pow(dx, 3.0);
    return position;
}

float CubicSpline1D::calculate_first_derivative(float x)
{
    if (x < this->x[0]) {
        return 0;
    } else if (x > this->x[this->x.size()-1]) {
        return 0;
    }
    int i = this->search_index(x);
    if (i < 0) {
        i = 0;
    }
    float dx = x - this->x[i];
    q31_t temp_c_q31 = this->c.get(i, 0);
    float temp_c;
    arm_q31_to_float(&temp_c_q31, &temp_c, 1);
    float dy = this->b[i] + 2.0 * temp_c * dx + 3.0 * this->d[i] * std::pow(dx, 2.0);
    return dy;
}

float CubicSpline1D::calculate_second_derivative(float x)
{
    if (x < this->x[0]) {
        return 0;
    } else if (x > this->x[this->x.size()-1]) {
        return 0;
    }

    int i = this->search_index(x);
    float dx = x - this->x[i];
    q31_t temp_c_q31 = this->c.get(i, 0);
    float temp_c;
    arm_q31_to_float(&temp_c_q31, &temp_c, 1);
    float ddy = 2.0 * temp_c + 6.0 * this->d[i] * dx;
    return ddy;
}

int CubicSpline1D::search_index(float x)
{
    auto itr = std::lower_bound(this->x.begin(), this->x.end(), x);
    int result = std::distance(this->x.begin(), itr) - 1;
    if (result < 0)
        return 0;
    return result;
    // return std::distance(this->x.begin(), itr) - 1;
}

ModelMatrix CubicSpline1D::calculate_a(std::vector<float> diff_x)
{
    int nx = this->x.size();
    ModelMatrix mat_a = ModelMatrix::zero(nx, nx);
    q31_t temp;
    mat_a.set(0, 0, 1.0);

    for (int i = 0; i < nx - 1; i++) {
        if (i != nx - 2) {
            float ele = 2.0 * (diff_x[i] + diff_x[i + 1]);
            arm_float_to_q31(&ele, &temp, 1);
            mat_a.set(i + 1, i + 1, temp);
        }
        arm_float_to_q31(&diff_x[i], &temp, 1);
        mat_a.set(i + 1, i, temp);
        mat_a.set(i, i + 1, temp);
    }

    mat_a.set(0, 1, 0);
    mat_a.set(nx - 1, nx - 2, 0);
    mat_a.set(nx - 1, nx - 1, 1 * 32767);
    return mat_a;
}

ModelMatrix CubicSpline1D::calculate_b(std::vector<float> diff_x, std::vector<float> coeff_a)
{
    int nx = this->x.size();
    ModelMatrix mat_b = ModelMatrix::zero(nx, 1);
    q31_t temp;

    for (int i = 0; i < nx - 2; i++) {
        float ele = 3.0 * (coeff_a[i + 2] - coeff_a[i + 1]) / diff_x[i + 1] - 3.0 * (coeff_a[i + 1] - coeff_a[i]) / diff_x[i];
        arm_float_to_q31(&ele, &temp, 1);
        mat_b.set(i + 1, 0, temp);
    }

    return mat_b;
}


CubicSpline2D::CubicSpline2D(std::vector<WayPoint> waypoints)
{
    std::vector<float> x;
    std::vector<float> y;

    for (uint32_t i = 0; i < waypoints.size(); i++) {
        x.push_back(waypoints[i].x);
        y.push_back(waypoints[i].y);
    }

    this->s = this->calculate_s(x, y);
    this->sx = new CubicSpline1D(this->s, x);
    this->sy = new CubicSpline1D(this->s, y);
}

CubicSpline2D::~CubicSpline2D()
{

}

std::vector<Point> CubicSpline2D::generate_spline_course(float speed, float ds)
{
    std::vector<Point> points;
    // calc_spline_course //
    float last_s = this->s[this->s.size() - 1]; // 최종 변위량
    for (float i = 0.0; i < last_s; i += ds) {
        Point point;
        this->calculate_position(i, &point.x, &point.y);

        point.yaw = this->calculate_yaw(i);

        point.k = this->calculate_curvature(i);

        if (isnan(point.k)) {
            point.k = 0;
        }

        points.push_back(point);
    }

    // calculate speed propfile //
    bool direction = true;
    float past_speed = speed;
    for (uint32_t i = 0; i < points.size() - 1; i++) {
        float dx = points[i + 1].x - points[i].x;
        float dy = points[i + 1].y - points[i].y;

        float move_direction = std::atan2(dy, dx);
        float current_gradient = dy/dx;
        float target_speed = speed;

        if (dx != 0.0 && dy != 0.0) {
            float dangle = std::abs(pi_2_pi(move_direction - points[i].yaw)); // angle
            if (dangle >= M_PI / 4.0) {
                direction = false;
            } else {
                direction = true;
                //target_speed = speed * (1 - abs(dangle) * 4.0 / M_PI) * 0.5 + past_speed * 0.5;
            }
        }
        if (direction) {
            points[i].speed = target_speed;
        } else {
            points[i].speed = -target_speed;
        }
        past_speed = target_speed;
    }
    points[points.size() - 1].speed = 0.0;

    return points;
}

void CubicSpline2D::calculate_position(float s, float* x, float* y)
{
    *x = this->sx->calculate_position(s);
    *y = this->sy->calculate_position(s);
}

float CubicSpline2D::calculate_curvature(float s)
{
    float dx = this->sx->calculate_first_derivative(s);
    float ddx = this->sx->calculate_second_derivative(s);
    float dy = this->sy->calculate_first_derivative(s);
    float ddy = this->sy->calculate_second_derivative(s);
    float k = (ddy * dx - ddx * dy) / std::pow(std::pow(dx, 2) + std::pow(dy, 2), (3.0 / 2.0));
    return k;
}

float CubicSpline2D::calculate_yaw(float s)
{
    float dx = this->sx->calculate_first_derivative(s);
    float dy = this->sy->calculate_first_derivative(s);
    float yaw = std::atan2(dy, dx);
    return yaw;
}

// 점과 점 거리 계산 후 ds에 저장, s에는 거리 누적값 저장
std::vector<float> CubicSpline2D::calculate_s(std::vector<float> x, std::vector<float> y)
{
    std::vector<float> s;
    float hypot_sum = 0.0;

    s.push_back(0.0);
    this->ds.clear();
    for (uint32_t i = 0; i < x.size() - 1; i++) {
        float diff_x = x[i + 1] - x[i];
        float diff_y = y[i + 1] - y[i];

        float hypot = std::sqrt(diff_x * diff_x + diff_y * diff_y);
        this->ds.push_back(hypot);

        hypot_sum += hypot;
        s.push_back(hypot_sum);
    }

    return s;
}

float pi_2_pi(float angle)
{
    while (angle > M_PI) {
        angle = angle - 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle = angle + 2.0 * M_PI;
    }
    return angle;
}
