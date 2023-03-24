#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <fstream>
#include <string>

#include <lqr_steer_control.h>
#include <cubic_spline_planner.h>


int main(int argc, const char * argv[])
{
    std::vector<WayPoint> waypoints;
    lqr_steer_control golfcar_path_tracker;
    ControlState current_state;

    std::cout << "start" << std::endl;

    waypoints.push_back(WayPoint(0.00, 0.00));
    waypoints.push_back(WayPoint(1.70, 2.70));
    waypoints.push_back(WayPoint(4.60, -0.20));
    waypoints.push_back(WayPoint(8.40, 2.70));
    waypoints.push_back(WayPoint(11.20, 5.30));
    waypoints.push_back(WayPoint(11.00, 8.10));
    waypoints.push_back(WayPoint(12.10, 10.20));
    waypoints.push_back(WayPoint(9.40, 12.80));
    waypoints.push_back(WayPoint(7.40, 11.30));
    waypoints.push_back(WayPoint(3.90, 14.10));
    waypoints.push_back(WayPoint(2.00, 12.10));
    waypoints.push_back(WayPoint(-0.50, 8.80));
    waypoints.push_back(WayPoint(-3.40, 7.50));
    waypoints.push_back(WayPoint(-6.50, 7.10));
    waypoints.push_back(WayPoint(-9.20, 3.80));
    waypoints.push_back(WayPoint(-9.70, 1.80));
    waypoints.push_back(WayPoint(-10.10, -0.70));
    waypoints.push_back(WayPoint(-11.50, -3.10));
    waypoints.push_back(WayPoint(-8.80, -4.30));
    waypoints.push_back(WayPoint(-6.30, -5.70));
    waypoints.push_back(WayPoint(-3.70, -6.80));
    waypoints.push_back(WayPoint(-0.80, -6.00));
    waypoints.push_back(WayPoint(1.60, -4.30));
    waypoints.push_back(WayPoint(0.00, 0.00));

    std::cout << "test1" << std::endl;

    CubicSpline2D spline_gener(waypoints);
    std::cout << "test2" << std::endl;
    std::vector<Point> splined_points = spline_gener.generate_spline_course(2, 0.1);
    std::cout << "test3" << std::endl;

    std::ofstream spline_list;
    spline_list.open("test.csv");

    for_each(splined_points.begin(), splined_points.end(), 
        [&spline_list](Point temp){
            std::cout << temp.x << ", " << temp.y << ", " << std::endl;
            spline_list << std::to_string(temp.x) <<  "\t" << std::to_string(temp.y) << "\n";
        });

    spline_list.close();
    golfcar_path_tracker.add_course(current_state, splined_points);

    return 0;
}

