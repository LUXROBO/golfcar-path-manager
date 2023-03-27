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
    lqr_steer_control golfcar_path_tracker;
    ControlState current_state(0, 0, 0, 0, 0);

    std::cout << "start" << std::endl;

    std::vector<WayPoint> waypoints;
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

    // generate spline
    CubicSpline2D spline_gener(waypoints);
    std::vector<Point> splined_points = spline_gener.generate_spline_course(2, 0.1);

    std::cout << "generating spline complete\n" << "size = " << splined_points.size() << std::endl;

    std::ofstream spline_list;
    std::ofstream move_path;
    spline_list.open("spline_list.csv");
    move_path.open("move_path.csv");

    for_each(splined_points.begin(), splined_points.end(), 
        [&spline_list](Point temp){
            // std::cout << temp.x << ", " << temp.y << ", " << std::endl;
            spline_list << std::to_string(temp.x) <<  "," << std::to_string(temp.y) << "\n";
        });
    spline_list.close();

    golfcar_path_tracker.set_state(current_state);

    // add course to lqr path tracker
    golfcar_path_tracker.add_course(current_state, splined_points);

    std::cout << "lqr test start" << std::endl;
    while (true) {
        try {
        // run lqr
            if (golfcar_path_tracker.update(0.01)) {
                std::cout << "finish test" << std::endl;
                return 0;
            } else {
                    static int progress_signal = 0;
                    if (progress_signal >= 10) {
                        std::cout << std::to_string(golfcar_path_tracker.get_state().x) <<  "," << std::to_string(golfcar_path_tracker.get_state().y) << std::endl;
                        progress_signal = 0;
                    } else {
                        progress_signal++;
                    }
                move_path << std::to_string(golfcar_path_tracker.get_state().x) <<  "," << std::to_string(golfcar_path_tracker.get_state().y) << "\n";
            }
        } catch (const std::exception &e) {
            std::cout << "error" << std::endl;
            return 0;
        }
    }
    move_path.close();
    

    return 0;
}

