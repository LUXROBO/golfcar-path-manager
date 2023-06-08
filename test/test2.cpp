#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <fstream>
#include <string>
#include <ctime>
#include <sstream>

#include "qformat_c.h"

#include <lqr_steer_control.h>
#include <pid_steer_control.h>
#include <cubic_spline_planner.h>


int main(int argc, const char * argv[])
{
    std::vector<WayPoint> waypoints;
    waypoints.push_back(WayPoint(0.00, 0.00));
    waypoints.push_back(WayPoint(10.00, 0.00));
    waypoints.push_back(WayPoint(0.00, 10.00));
    waypoints.push_back(WayPoint(-10.00, 0.00));
    waypoints.push_back(WayPoint(0.00, -10.00));
    waypoints.push_back(WayPoint(10.00, 0.00));
    waypoints.push_back(WayPoint(0.00, 0.00));

    // generate spline
    CubicSpline2D spline_gener(waypoints);
    std::vector<Point> splined_points = spline_gener.generate_spline_course(2, 1);
    std::ofstream spline_list;
    spline_list.open("spline_list.csv");

    for_each(splined_points.begin(), splined_points.end(),
        [&spline_list](Point temp){
            std::cout << temp.x << ", " << temp.y << std::endl;
            spline_list << std::to_string(temp.x) <<  "," << std::to_string(temp.y) << "\n";
        });
    spline_list.close();

    return 0;
}

