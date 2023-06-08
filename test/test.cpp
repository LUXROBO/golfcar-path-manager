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

static std::vector<std::string> string_split(std::string raw_string, std::string edge)
{
    std::vector<std::string> result;
    size_t past_index = 0;
    while (true) {
        size_t index = raw_string.find(edge, past_index);
        if (index == std::string::npos) {
            result.push_back(raw_string.substr(past_index));
            return result;
        }
        std::string splited_string = raw_string.substr(past_index, index - past_index);
        if (splited_string.size() == 0) {
            splited_string = "NOPE";
        }
        result.push_back(splited_string);
        past_index = index + 1;
    }
}


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
    std::ofstream move_path;
    std::ofstream error_measure;
    std::ifstream spline_list;

    lqr_steer_control golfcar_path_tracker;
    ControlState current_state(0, 0, 0, 0, 0);
    golfcar_path_tracker.init(45.0 * M_PI / 180.0, 10.0 / 3.6, 2.15, 1);

    std::cout << "start" << std::endl;

    std::vector<WayPoint> waypoints;
    // waypoints.push_back(WayPoint(0.00, 0.00));
    // waypoints.push_back(WayPoint(10.00, 0.00));
    // waypoints.push_back(WayPoint(0.00, 10.00));
    // waypoints.push_back(WayPoint(-10.00, 0.00));
    // waypoints.push_back(WayPoint(0.00, -10.00));
    // waypoints.push_back(WayPoint(10.00, 0.00));
    // waypoints.push_back(WayPoint(0.00, 0.00));

    spline_list.open("spline_kwang_woon.csv");
    // char spline_string_data[100] = "";
    std::string spline_string_data = "";
    std::vector<Point> splined_points;
    while (!spline_list.eof()) {
        spline_list >> spline_string_data;
        std::vector<std::string> splited_data = string_split(spline_string_data, ",");
        double x = std::stod(splited_data[0]);
        double y = std::stod(splited_data[1]);
        double yaw = std::stod(splited_data[2]);
        double v = std::stod(splited_data[3]);
        double k = std::stod(splited_data[4]);
        // std::cout << x << " " << y << " " << yaw << " " << v << " " << k << std::endl;
        splined_points.push_back(Point{x, y, yaw, k, v});
    }
    spline_list.close();
    std::cout << "generating spline complete\n" << "size = " << splined_points.size() << std::endl;

    time_t timer = time(NULL);
    struct tm* t = localtime(&timer);
    std::string time_string = std::to_string(t->tm_mday) + "-" + std::to_string(t->tm_hour) + "-" + std::to_string(t->tm_min) + "-" + std::to_string(t->tm_sec) + ".csv";
    move_path.open("move_path_" + time_string, std::ofstream::out);

    std::vector<Point> splined_points_cut;
    if (splined_points.size() <= 256) {
        splined_points_cut = splined_points;
        splined_points.clear();
    } else {
        splined_points_cut.assign(splined_points.begin(), splined_points.begin() + 256);
        splined_points.erase(splined_points.begin(), splined_points.begin() + 256);
    }
    golfcar_path_tracker.set_state(current_state);
    // add course to lqr path tracker
    golfcar_path_tracker.set_course(current_state, splined_points_cut);

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
            if ((golfcar_path_tracker.get_remain_point() < 128) && (splined_points.size() != 0)) {
                std::cout << "new" << std::endl;
                if (splined_points.size() <= 128) {
                    splined_points_cut = splined_points;
                    splined_points.clear();
                } else {
                    splined_points_cut.assign(splined_points.begin(), splined_points.begin() + 128);
                    splined_points.erase(splined_points.begin(), splined_points.begin() + 128);
                }
                golfcar_path_tracker.remove_points(128);
                golfcar_path_tracker.add_course(golfcar_path_tracker.get_state(), splined_points_cut);
                std::cout << "new complete" << std::endl;
            } 
            if (golfcar_path_tracker.update(0.1)) {
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

                // error_list.clear();
                // error_average = 0;
                // // golfcar_path_tracker = pid_steer_control();
                // golfcar_path_tracker = lqr_steer_control();
                // memset((void*)&current_state, 0, sizeof(ControlState));
                // golfcar_path_tracker.set_state(current_state);
                // golfcar_path_tracker.add_course(current_state, splined_points);

                return 0;
            } else {
                static int progress_signal = 0;
                if (progress_signal >= 10) {
                    std::cout << golfcar_path_tracker.get_state().x << "  " << golfcar_path_tracker.get_state().y << " " << golfcar_path_tracker.get_target_index() << std::endl;
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

