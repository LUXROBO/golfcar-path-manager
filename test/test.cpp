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
    // ModelMatrix A(3, 3);
    // ModelMatrix B(3, 1);
    // for (int i=0; i<3; i++) {
    //     for (int j=0; j<3; j++) {
    //         A.set(i, j, j + 3*i) ;
    //     }
    // }
    // for (int i=0; i<3; i++) {
    //     B.set(i, 0, i) ;
    // }

    // std::cout << "A = \n";
    // for (int i=0; i<A.row(); i++) {
    //     for (int j=0; j<A.column(); j++) {
    //         std::cout << A.get(i, j).to_double() << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // std::cout << "B = \n";
    // for (int i=0; i<B.row(); i++) {
    //     for (int j=0; j<B.column(); j++) {
    //         std::cout << B.get(i, j).to_double() << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // ModelMatrix C = A.inverse();
    // std::cout << "C = \n";
    // for (int i=0; i<C.row(); i++) {
    //     for (int j=0; j<C.column(); j++) {
    //         std::cout << C.get(i, j).to_double() << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // ModelMatrix D = A * C;
    // std::cout << "D = \n";
    // for (int i=0; i<D.row(); i++) {
    //     for (int j=0; j<D.column(); j++) {
    //         std::cout << D.get(i, j).to_double() << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // return 0;


    lqr_steer_control golfcar_path_tracker;
    golfcar_path_tracker.init(0.785398f, 10.0 / 3.6, 2.15, 0.6);
    ControlState current_state(0, 0, 0, 0, 0);

    std::cout << "start" << std::endl;

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

                if (loop_count >= 1000) {
                    std::cout << "min_var : " << min_var << " min_err : " << min_err << " max_err : " << max_err << " gain : " << selected_gain << std::endl;

                    // golfcar_path_tracker = pid_steer_control();
                    golfcar_path_tracker = lqr_steer_control();

                    memset((void*)&current_state, 0, sizeof(ControlState));
                    golfcar_path_tracker.set_state(current_state);
                    golfcar_path_tracker.add_course(current_state, splined_points);
                    while (!golfcar_path_tracker.update(0.01)) {
                        move_path << std::to_string(golfcar_path_tracker.get_state().x) <<  "," << std::to_string(golfcar_path_tracker.get_state().y) << "\n";
                    }
                    move_path.close();
                    return 0;
                }

                error_list.clear();
                error_average = 0;
                // golfcar_path_tracker = pid_steer_control();
                golfcar_path_tracker = lqr_steer_control();
                memset((void*)&current_state, 0, sizeof(ControlState));
                golfcar_path_tracker.set_state(current_state);
                golfcar_path_tracker.add_course(current_state, splined_points);
            } else {
                static int progress_signal = 0;
                if (progress_signal >= 10) {
                    std::cout << golfcar_path_tracker.get_state().x << " " << golfcar_path_tracker.get_state().y << std::endl;
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

