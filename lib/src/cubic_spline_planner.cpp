#include "cubic_spline_planner.h"

#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <iostream>


CubicSpline1D::CubicSpline1D(std::vector<double> x, std::vector<double> y)
{
    std::vector<q_format> diff_x;
    for (uint32_t i = 0; i < x.size() - 1; i++) {
        double diff = x[i + 1] - x[i];
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
    this->c = coeff_a.inverse(0.001) * coeff_b;

    std::cout << "D = \n";
    for (int i=0; i<this->c.row(); i++) {
        for (int j=0; j<this->c.column(); j++) {
            std::cout << this->c.get(i, j).to_double() << " ";
        }
        std::cout << std::endl;
    }

    for (uint32_t i = 0; i < this->x.size() - 1; i++) {
        double d_temp = ((this->c.get(i + 1, 0) - this->c.get(i, 0)) / (3.0 * diff_x[i])).to_double();
        double b_temp = (1.0 / diff_x[i] * (this->a[i + 1] - this->a[i]) - diff_x[i] / 3.0 * (2.0 * this->c.get(i, 0) + this->c.get(i + 1, 0))).to_double();
        this->d.push_back(d_temp);
        this->b.push_back(b_temp);
    }
}

CubicSpline1D::~CubicSpline1D()
{

}

double CubicSpline1D::calculate_position(double x)
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
    double dx = x - this->x[i];
    q_format position = this->a[i] + this->b[i] * dx + this->c.get(i, 0) * std::pow(dx, 2.0) + this->d[i] * std::pow(dx, 3.0);
    return position.to_double();
}

double CubicSpline1D::calculate_first_derivative(double x)
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
    double dx = x - this->x[i];
    q_format dy = this->b[i] + 2.0 * this->c.get(i, 0) * dx + 3.0 * this->d[i] * std::pow(dx, 2.0);
    return dy.to_double();
}

double CubicSpline1D::calculate_second_derivative(double x)
{
    if (x < this->x[0]) {
        return 0;
    } else if (x > this->x[this->x.size()-1]) {
        return 0;
    }

    int i = this->search_index(x);
    double dx = x - this->x[i];
    q_format ddy = 2.0 * this->c.get(i, 0) + 6.0 * this->d[i] * dx;
    return ddy.to_double();
}

int CubicSpline1D::search_index(double x)
{
    auto itr = std::lower_bound(this->x.begin(), this->x.end(), x);
    int result = std::distance(this->x.begin(), itr) - 1;
    if (result < 0)
        return 0;
    return result;
    // return std::distance(this->x.begin(), itr) - 1;
}

ModelMatrix CubicSpline1D::calculate_a(std::vector<q_format> diff_x)
{
    int nx = this->x.size();
    ModelMatrix mat_a = ModelMatrix::zero(nx, nx);
    mat_a.set(0, 0, 1.0);

    for (int i = 0; i < nx - 1; i++) {
        if (i != nx - 2) {
            q_format ele = 2.0 * (diff_x[i] + diff_x[i + 1]);
            mat_a.set(i + 1, i + 1, ele);
        }
        mat_a.set(i + 1, i, diff_x[i]);
        mat_a.set(i, i + 1, diff_x[i]);
    }

    mat_a.set(0, 1, 0.0);
    mat_a.set(nx - 1, nx - 2, 0.0);
    mat_a.set(nx - 1, nx - 1, 1.0);
    return mat_a;
}

ModelMatrix CubicSpline1D::calculate_b(std::vector<q_format> diff_x, std::vector<double> coeff_a)
{
    int nx = this->x.size();
    ModelMatrix mat_b = ModelMatrix::zero(nx, 1);

    for (int i = 0; i < nx - 2; i++) {
        q_format ele = 3.0 * (coeff_a[i + 2] - coeff_a[i + 1]) / diff_x[i + 1] - 3.0 * (coeff_a[i + 1] - coeff_a[i]) / diff_x[i];
        mat_b.set(i + 1, 0, ele);
    }

    return mat_b;
}


CubicSpline2D::CubicSpline2D(std::vector<WayPoint> waypoints)
{
    std::vector<double> x;
    std::vector<double> y;

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

std::vector<Point> CubicSpline2D::generate_spline_course(double speed, double ds)
{
    std::vector<Point> points;

    // calc_spline_course //
    double last_s = this->s[this->s.size() - 1]; // 최종 변위량
    for (double i = 0.0; i < last_s; i += ds) {
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
    double past_speed = speed;
    for (uint32_t i = 0; i < points.size() - 1; i++) {
        double dx = points[i + 1].x - points[i].x;
        double dy = points[i + 1].y - points[i].y;

        double move_direction = std::atan2(dy, dx);
        double current_gradient = dy/dx;
        double target_speed = speed;

        if (dx != 0.0 && dy != 0.0) {
            double dangle = std::abs(pi_2_pi(move_direction - points[i].yaw)); // angle
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

void CubicSpline2D::calculate_position(double s, double* x, double* y)
{
    *x = this->sx->calculate_position(s);
    *y = this->sy->calculate_position(s);
}

double CubicSpline2D::calculate_curvature(double s)
{
    double dx = this->sx->calculate_first_derivative(s);
    double ddx = this->sx->calculate_second_derivative(s);
    double dy = this->sy->calculate_first_derivative(s);
    double ddy = this->sy->calculate_second_derivative(s);
    double k = (ddy * dx - ddx * dy) / std::pow(std::pow(dx, 2) + std::pow(dy, 2), (3.0 / 2.0));
    return k;
}

double CubicSpline2D::calculate_yaw(double s)
{
    double dx = this->sx->calculate_first_derivative(s);
    double dy = this->sy->calculate_first_derivative(s);
    double yaw = std::atan2(dy, dx);
    return yaw;
}

// 점과 점 거리 계산 후 ds에 저장, s에는 거리 누적값 저장
std::vector<double> CubicSpline2D::calculate_s(std::vector<double> x, std::vector<double> y)
{
    std::vector<double> s;
    double hypot_sum = 0.0;

    s.push_back(0.0);
    this->ds.clear();
    for (uint32_t i = 0; i < x.size() - 1; i++) {
        double diff_x = x[i + 1] - x[i];
        double diff_y = y[i + 1] - y[i];

        double hypot = std::sqrt(diff_x * diff_x + diff_y * diff_y);
        this->ds.push_back(hypot);

        hypot_sum += hypot;
        s.push_back(hypot_sum);
    }

    return s;
}

double pi_2_pi(double angle)
{
    while (angle > M_PI) {
        angle = angle - 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle = angle + 2.0 * M_PI;
    }
    return angle;
}
