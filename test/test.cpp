#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <fstream>
#include <string>
#include <ctime>
#include <sstream>
#include <random>

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
    lqr_steer_control golfcar_path_tracker;
    
    ControlState current_state(0, 0, 0, 0, 0);

    std::cout << "start" << std::endl;

    std::ifstream spline_list;
    std::string waypoints_str;
    spline_list.open("./spline_kwang_woon.csv");


    std::vector<Point> splined_points;
    while (std::getline(spline_list, waypoints_str)){
        float x_f = 0;
        float y_f = 0;
        float yaw_f = 0;
        float v_f = 0;
        float k_f = 0;
        sscanf(waypoints_str.c_str(),"%f,%f,%f,%f,%f\n", &x_f, &y_f, &yaw_f, &k_f, &v_f);
        Point ttt = {x_f, y_f, yaw_f, v_f, k_f};
        splined_points.push_back(ttt);
    }

    std::cout << "generating spline complete\n" << "size = " << splined_points.size() << std::endl;

    time_t timer = time(NULL);
    struct tm* t = localtime(&timer);

    std::string time_string = std::to_string(t->tm_mday) + "-" + std::to_string(t->tm_hour) + "-" + std::to_string(t->tm_min) + "-" + std::to_string(t->tm_sec) + ".csv";

    std::ofstream move_path;
    std::ofstream error_measure;
    spline_list.open("spline_list.csv");
    move_path.open("move_path_" + time_string, std::ofstream::out);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(1, 200);

    golfcar_path_tracker.set_q(0,0,18.3);
    golfcar_path_tracker.set_q(1,1,0.1);
    golfcar_path_tracker.set_q(2,2,1.5);
    golfcar_path_tracker.set_q(3,3,3.9);
    golfcar_path_tracker.set_r(0,0,1.4);

    double q[4] = {0,};
    double r[1] = {0,};

    std::cout << "lqr test start" << std::endl;
    for (int i=0; i<1; i++) {
        std::vector<double> error_list;
        double error_average = 0;

        static double min_var = 100;
        double max_err = 0;
        double min_err = 0;
        double selected_gain = 0;
        current_state.x = splined_points[0].x;
        current_state.y = splined_points[0].y;
        current_state.v = splined_points[0].speed;
        current_state.steer = 0;
        current_state.yaw = splined_points[0].yaw;
        golfcar_path_tracker.init(0.785398f, 10.0 / 3.6, 2.15, 0.6);
        golfcar_path_tracker.set_state(current_state);
        // add course to lqr path tracker
        golfcar_path_tracker.add_course(current_state, splined_points);

        // for (int j=0; j<4; j++) {
        //     golfcar_path_tracker.set_q(j,j,(double)dis(gen) / 10);
        // }
        // golfcar_path_tracker.set_r(0,0,(double)dis(gen) / 10);

        while (true) {
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
                    std::cout << "Q : ";
                    for (int j=0; j<4; j++) {
                        q[j] = golfcar_path_tracker.get_q(j,j);
                        std::cout << golfcar_path_tracker.get_q(j,j) << " ";
                    }
                    r[0] = golfcar_path_tracker.get_r(0,0);
                    std::cout << "R : " << golfcar_path_tracker.get_r(0,0) << std::endl;
                    std::cout << "min_var : " << min_var << " min_err : " << min_err << " max_err : " << max_err << " gain : " << selected_gain << std::endl;
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
                break;
                // return 0;
            } else {
                static int progress_signal = 0;
                if (progress_signal >= 10) {
                    // std::cout << golfcar_path_tracker.get_state().x << " " << golfcar_path_tracker.get_state().y << std::endl;
                    // move_path << std::to_string(golfcar_path_tracker.get_state().x) <<  "," << std::to_string(golfcar_path_tracker.get_state().y) << "\n";
                    if (golfcar_path_tracker.get_target_index() != 0) {
                        auto now_point = Point{golfcar_path_tracker.get_state().x, golfcar_path_tracker.get_state().y, 0, 0, 0};
                        double error_amount = distance_between_point_and_line(now_point, splined_points[golfcar_path_tracker.get_target_index()-1], splined_points[golfcar_path_tracker.get_target_index()]);
                        error_list.push_back(error_amount);
                        error_average += error_amount;
                    }
                    progress_signal = 0;
                } else {
                    progress_signal++;
                }
                // move_path << std::to_string(golfcar_path_tracker.get_state().x) <<  "," << std::to_string(golfcar_path_tracker.get_state().y) << "\n";
            }
        }
    }

    std::cout << "finalQ : ";
    for (int j=0; j<4; j++) {
        std::cout << q[j] << " ";
    }
    std::cout << "finalR : " << r[0] << std::endl;
    // move_path.close();
    return 0;
}

