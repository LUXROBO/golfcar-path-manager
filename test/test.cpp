#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <fstream>
#include <string>
#include <ctime>
#include <sstream>

#include <lqr_steer_control.h>
#include <pid_steer_control.h>
#include <cubic_spline_planner.h>


static double distance_between_point_and_line(Point point, Point line_point1, Point line_point2)
{
    double a = (line_point1.y - line_point2.y) / (line_point1.x - line_point2.x);
    double c = line_point1.y - a * line_point1.x;
    double b = -1;

    return abs(a * point.x + b * point.y + c) / sqrt(a * a + b * b);
}

int main(int argc, const char * argv[])
{
    // pid_steer_control golfcar_path_tracker;
    lqr_steer_control golfcar_path_tracker;
    ControlState current_state(0, 0, 0, 0, 0);
    golfcar_path_tracker.init(45.0 * M_PI / 180.0, 10.0 / 3.6, 2.15, 1);

    std::cout << "start" << std::endl;

    std::vector<WayPoint> waypoints;
    // waypoints.push_back(WayPoint(0.00, 0.00));
    // waypoints.push_back(WayPoint(1.70, 2.70));
    // waypoints.push_back(WayPoint(4.60, -0.20));
    // waypoints.push_back(WayPoint(8.40, 2.70));
    // waypoints.push_back(WayPoint(11.20, 5.30));
    // waypoints.push_back(WayPoint(11.00, 8.10));
    // waypoints.push_back(WayPoint(12.10, 10.20));
    // waypoints.push_back(WayPoint(9.40, 12.80));
    // waypoints.push_back(WayPoint(7.40, 11.30));
    // waypoints.push_back(WayPoint(3.90, 14.10));
    // waypoints.push_back(WayPoint(2.00, 12.10));
    // waypoints.push_back(WayPoint(-0.50, 8.80));
    // waypoints.push_back(WayPoint(-3.40, 7.50));
    // waypoints.push_back(WayPoint(-6.50, 7.10));
    // waypoints.push_back(WayPoint(-9.20, 3.80));
    // waypoints.push_back(WayPoint(-9.70, 1.80));
    // waypoints.push_back(WayPoint(-10.10, -0.70));
    // waypoints.push_back(WayPoint(-11.50, -3.10));
    // waypoints.push_back(WayPoint(-8.80, -4.30));
    // waypoints.push_back(WayPoint(-6.30, -5.70));
    // waypoints.push_back(WayPoint(-3.70, -6.80));
    // waypoints.push_back(WayPoint(-0.80, -6.00));
    // waypoints.push_back(WayPoint(1.60, -4.30));
    // waypoints.push_back(WayPoint(0.00, 0.00));

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

    std::cout << "generating spline complete\n" << "size = " << splined_points.size() << std::endl;

    time_t timer = time(NULL);
    struct tm* t = localtime(&timer);

    std::string time_string = std::to_string(t->tm_mday) + "-" + std::to_string(t->tm_hour) + "-" + std::to_string(t->tm_min) + "-" + std::to_string(t->tm_sec) + ".csv";

    std::ofstream spline_list;
    std::ofstream move_path;
    std::ofstream error_measure;
    spline_list.open("spline_list.csv");
    move_path.open("move_path_" + time_string, std::ofstream::out);

    for_each(splined_points.begin(), splined_points.end(),
        [&spline_list](Point temp){
            std::cout << temp.x << ", " << temp.y << std::endl;
            spline_list << std::to_string(temp.x) <<  "," << std::to_string(temp.y) << "\n";
        });
    spline_list.close();

    golfcar_path_tracker.set_state(current_state);
    // add course to lqr path tracker
    golfcar_path_tracker.add_course(current_state, splined_points);

    std::vector<double> error_list;
    double error_average = 0;

    double min_var = 100;
    double max_err = 0;
    double min_err = 0;
    double selected_gain = 0;

    std::cout << "lqr test start" << std::endl;
    while (true) {
        try {
        // run lqr
            if (golfcar_path_tracker.update(0.01)) {
                static int loop_count = 1;
                std::cout << "finish test " << loop_count++ << std::endl;

                error_average /= error_list.size();
                double variance = 0;
                for (auto x : error_list) {
                    variance += pow(x - error_average, 2);
                }
                variance /= error_list.size();
                if (min_var > variance) {
                    min_var = variance;
                    max_err = *max_element(error_list.begin(), error_list.end());
                    min_err = *min_element(error_list.begin(), error_list.end());
                    selected_gain = loop_count * 0.1;
                }

                // if (loop_count >= 1000) {
                //     std::cout << "min_var : " << min_var << " min_err : " << min_err << " max_err : " << max_err << " gain : " << selected_gain << std::endl;

                //     // golfcar_path_tracker = pid_steer_control();
                //     golfcar_path_tracker = lqr_steer_control();

                //     memset((void*)&current_state, 0, sizeof(ControlState));
                //     golfcar_path_tracker.set_state(current_state);
                //     golfcar_path_tracker.add_course(current_state, splined_points);
                //     while (!golfcar_path_tracker.update(0.01)) {
                //         move_path << std::to_string(golfcar_path_tracker.get_state().x) <<  "," << std::to_string(golfcar_path_tracker.get_state().y) << "\n";
                //     }
                //     move_path.close();
                //     return 0;
                // }

                error_list.clear();
                error_average = 0;
                // golfcar_path_tracker = pid_steer_control();
                golfcar_path_tracker = lqr_steer_control();
                memset((void*)&current_state, 0, sizeof(ControlState));
                golfcar_path_tracker.set_state(current_state);
                golfcar_path_tracker.add_course(current_state, splined_points);

                return 0;
            } else {
                static int progress_signal = 0;
                if (progress_signal >= 10) {
                    std::cout << golfcar_path_tracker.get_state().x << "  " << golfcar_path_tracker.get_state().y << std::endl;
                    move_path << std::to_string(golfcar_path_tracker.get_state().x) <<  "," << std::to_string(golfcar_path_tracker.get_state().y) << "\n";
                    // if (golfcar_path_tracker.target_ind != 0) {
                    //     auto now_point = Point{golfcar_path_tracker.get_state().x, golfcar_path_tracker.get_state().y, 0, 0, 0};
                    //     double error_amount = distance_between_point_and_line(now_point, splined_points[golfcar_path_tracker.target_ind-1], splined_points[golfcar_path_tracker.target_ind]);
                    //     error_list.push_back(error_amount);
                    //     error_average += error_amount;
                    // }
                    progress_signal = 0;
                } else {
                    progress_signal++;
                }
                // move_path << std::to_string(golfcar_path_tracker.get_state().x) <<  "," << std::to_string(golfcar_path_tracker.get_state().y) << "\n";
            }
        } catch (const std::exception &e) {
            std::cout << "error" << std::endl;
            return 0;
        }
    }
    move_path.close();
    return 0;
}

