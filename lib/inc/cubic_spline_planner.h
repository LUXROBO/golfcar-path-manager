#ifndef CUBIC_SPLINE_PLANNER_H
#define CUBIC_SPLINE_PLANNER_H

#include <math.h>
#include <vector>

#include "model_matrix.h"
#include "path_manager.h"


class CubicSpline1D
{
public:
    CubicSpline1D(std::vector<float> x, std::vector<float> y);
    ~CubicSpline1D();

public:
    float calculate_position(float x);
    float calculate_first_derivative(float x);
    float calculate_second_derivative(float x);

private:
    int search_index(float x);
    ModelMatrix calculate_a(std::vector<float> diff_x);
    ModelMatrix calculate_b(std::vector<float> diff_x, std::vector<float> coeff_a);

private:
    std::vector<float> x;
    std::vector<float> y;

    std::vector<float> a;
    std::vector<float> b;
    ModelMatrix c;
    std::vector<float> d;
};

class CubicSpline2D
{
public:
    CubicSpline2D(std::vector<WayPoint> waypoints);
    ~CubicSpline2D();

public:
    std::vector<Point> generate_spline_course(float speed, float ds=0.1);

private:
    void calculate_position(float s, float* x, float* y);
    float calculate_curvature(float s);
    float calculate_yaw(float s);
    std::vector<float> calculate_s(std::vector<float> x, std::vector<float> y);

private:
    std::vector<float> s;
    CubicSpline1D *sx;
    CubicSpline1D *sy;

    std::vector<float> ds;
};

float pi_2_pi(float angle);

#endif
