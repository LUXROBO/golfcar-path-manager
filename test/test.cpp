#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <fstream>
#include <string>
#include <ctime>
#include <sstream>
#include <random>
#include <typeinfo>

#include <lqr_steer_control.h>
#include <pid_steer_control.h>
#include <curvature_steer_control.h>

#define TRACK_POINT_SPLIT_MAX 256
#define TRACK_POINT_SPLIT_HALF 128

std::vector<path_point> spline_points;
int splined_points_cursor = 0;
double time_sum = 1;

std::vector<std::string> splitString(const std::string& input, char delimiter)
{
    std::vector<std::string> tokens;
    std::istringstream stream(input);
    std::string token;

    while (std::getline(stream, token, delimiter)) {
        tokens.push_back(token);
    }

    return tokens;
}

int main(int argc, const char * argv[])
{
    // path_tracker* tracker;
    curvature_steer_control tracker(PT_M_PI_2/2, 2.5, 2.18);

    // path_point_t current1 = {27.549156,-2.941727,0.604129, 0, 0};
    // path_point_t target1 = {28.307000,-1.742000,1.031300, 0, 0};
    // path_point_t current2 = {27.639037,-2.880933,0.586837, 0, 0};
    // path_point_t target2 = {28.307000,-1.742000,1.031300, 0, 0};
    // tracker.test_function(current1, target1);
    // tracker.test_function(current2, target2);
    // return 0;
    // tracker = &curvature_tracker;

    // std::string filePath = "../../../path_gps_smi_p_final.csv";path_new_map
    std::string filePath = "D:\\git\\git_luxrobo\\golfcart_vehicle_control_unit_stm32\\application\\User\\lib\\golfcar_lqr_path_manager\\path_gps_smi_p_final.csv";
    // std::string filePath = "D:\\git\\git_luxrobo\\golfcart_vehicle_control_unit_stm32\\application\\User\\lib\\golfcar_lqr_path_manager\\path_new_map.csv";
    std::ifstream inputFile(filePath);

    filePath = "log.csv";
    std::ofstream outputFile(filePath);
    outputFile.close();

    // 파일이 열렸는지 확인
    if (!inputFile.is_open()) {
        std::cerr << "can not open." << std::endl;
        return 1;
    }
    std::string line;
    while (std::getline(inputFile, line)) {
        std::vector<std::string> result = splitString(line, ',');
        path_point splined_point = {std::stod(result[0]), std::stod(result[1]), std::stod(result[2]), std::stod(result[3]), std::stod(result[4])};
        static path_point origin_splined_point = {0, 0, 0, 0, 0};
        if (spline_points.size() == 0) {
            origin_splined_point = splined_point;
            splined_point.x = 0;
            splined_point.y = 0;
        } else {
            splined_point.x -= origin_splined_point.x;
            splined_point.y -= origin_splined_point.y;
        }
        spline_points.push_back(splined_point);
    }
    inputFile.close();

    pt_control_state_t init_state = {spline_points[0].x, spline_points[0].y, spline_points[0].yaw, 0, 0};

    // std::vector<path_point_t> splined_points_size_cutting;
    // if (spline_points.size() < TRACK_POINT_SPLIT_MAX) {
    //     splined_points_size_cutting = spline_points;
    //     splined_points_cursor += spline_points.size();
    // } else {
    //     splined_points_size_cutting = std::vector<path_point_t>(spline_points.begin(), spline_points.begin() + TRACK_POINT_SPLIT_MAX);
    //     splined_points_cursor += TRACK_POINT_SPLIT_MAX;
    // }

    // tracker.set_path_points(init_state, splined_points_size_cutting);
    tracker.set_path_points(init_state, spline_points);


    // path_point current = {0, 0, 0, 0, 0};
    // path_point target = {2, 2, PT_M_PI_2, 0, 0};

    // path_point new_circle = tracker.test_function(current, target);
    // std::cout << new_circle.x << ", " << new_circle.y << ", " << new_circle.k << ", " << 1 / new_circle.k << ", " <<std::endl;

    while (true) {
        // if (tracker.get_remain_point_num() < TRACK_POINT_SPLIT_HALF && (splined_points_cursor != (int)spline_points.size())) {
        //     std::vector<path_point_t> splined_points_size_cutting;
        //     if (spline_points.size() - splined_points_cursor < TRACK_POINT_SPLIT_MAX) { // 남은 개수가 절반보다 남지 않은 경우
        //         splined_points_size_cutting = std::vector<path_point_t>(spline_points.begin() + splined_points_cursor, spline_points.end());
        //         splined_points_cursor = spline_points.size();
        //     } else {
        //         splined_points_size_cutting = std::vector<path_point_t>(spline_points.begin() + splined_points_cursor,
        //                                                             spline_points.begin() + splined_points_cursor + TRACK_POINT_SPLIT_MAX);
        //         splined_points_cursor += TRACK_POINT_SPLIT_HALF;
        //     }
        //     tracker.set_path_points(tracker.get_last_updated_state(), splined_points_size_cutting);
        //     // tracker.remove_points(TRACK_POINT_SPLIT_HALF);
        //     // tracker.add_course(tracker.get_last_updated_state(), splined_points_size_cutting);
        // }
        pt_update_result_t update_result = tracker.update(time_sum);
        time_sum += 0.01;
        if (update_result != PT_UPDATE_RESULT_RUNNING) {
            if (update_result == PT_UPDATE_RESULT_NOT_READY) {
                continue;
            }
            printf("finish %d", (int)update_result);
            return 0;
        } else {
            static int debug_count = 0;
            if (debug_count % 10 == 0) {
                filePath = "log.csv";
                outputFile = std::ofstream(filePath, std::ios::app);

                // 파일이 열렸는지 확인
                if (!outputFile.is_open()) {
                    std::cerr << "can not open." << std::endl;
                    return 1;
                }

                pt_control_state_t current_state = tracker.get_predict_state();
                char debug_string[200];
                int index = tracker.get_front_target_point_index();
                sprintf(debug_string, "%lf,%lf,%lf,%lf,%lf,,%lf,%lf,%lf,%d,,%lf,%lf,%lf",current_state.x,
                                                                    current_state.y,
                                                                    current_state.yaw,
                                                                    current_state.steer,
                                                                    tracker.get_yaw_error(),
                                                                    spline_points[index].x,
                                                                    spline_points[index].y,
                                                                    spline_points[index].yaw,
                                                                    index,
                                                                    tracker.past_path_circle.x,
                                                                    tracker.past_path_circle.y,
                                                                    1 / tracker.past_path_circle.k);
                outputFile << debug_string << "\n";
                // std::cout << debug_string << std::endl;
                outputFile.close();
            }
            debug_count++;
        }
    }


    return 0;
}

