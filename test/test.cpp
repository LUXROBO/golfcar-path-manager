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

std::vector<path_point_t> get_path(std::string file_name, int length, int cursor)
{
    std::ifstream inputFile(file_name);
    std::string line;
    std::vector<path_point_t> result;
    static path_point_t origin = {0, 0, 0, 0, 0};
    int flag = 0;

    if (!inputFile.is_open()) {
        std::cerr << "can not open." << std::endl;
        return std::vector<path_point_t>();
    }
    int line_num = 0;
    while (std::getline(inputFile, line)) {
        if (line_num++ < cursor) {
            continue;
        }
        std::vector<std::string> splited_line = splitString(line, ',');
        path_point splined_point = {std::stod(splited_line[0]), std::stod(splited_line[1]), std::stod(splited_line[2]), std::stod(splited_line[4]), std::stod(splited_line[3])};
        // if (cursor == 0 && flag == 0) {
        //     origin = splined_point;
        //     flag = 1;
        // }
        // splined_point.x -= origin.x;
        // splined_point.y -= origin.y;
        result.push_back(splined_point);
        if ((line_num - cursor + 1) >= length) {
            break;
        }
    }
    inputFile.close();
    return result;
}

int main(int argc, const char * argv[])
{
    // path_tracker* tracker;
    curvature_steer_control tracker(PT_M_PI_2/2, 2.5, 2.18);
    // pid_steer_control tracker(PT_M_PI_2/2, 2.5, 2.18);

    // path_point_t current1 = {-52186.522601,3998336.599293,-2.067044,0,0};
    // path_point_t current2 = {-52186.569003,3998336.510714,-2.041866,0,0};

    // path_point_t target1 = {-52186.667500,3998335.453700,-1.818900,0,0};
    // path_point_t target2 = {-52186.667500,3998335.453700,-1.818900,0,0};

    // current2.x -= current1.x;
    // current2.y -= current1.y;

    // target1.x -= current1.x;
    // target1.y -= current1.y;

    // target2.x -= current1.x;
    // target2.y -= current1.y;

    // current1.x = 0;
    // current1.y = 0;

    // tracker.test_function(current1, target1);
    // tracker.test_function(current2, target2);
    // return 0;

    std::string log_name = "log.csv";

    // std::string map_file_path = "../../../path_gps_smi_p_final.csv";path_new_map   path_smi_new_mrp2000_7km   path_smi_new_mrp2000_7km path_debug
    std::string map_file_path = "D:\\git\\git_luxrobo\\golfcart_vehicle_control_unit_stm32\\application\\User\\lib\\golfcar_lqr_path_manager\\path_cl.csv";
    // std::string map_file_path = "D:\\git\\git_luxrobo\\golfcart_vehicle_control_unit_stm32\\application\\User\\lib\\golfcar_lqr_path_manager\\path_new_map.csv";

    std::ofstream outputFile(log_name);
    int count = 1;
    while (!outputFile.is_open()) {
        outputFile.close();

        log_name.clear();

        std::ostringstream oss;
        oss << count++ << "log.csv";
        log_name = oss.str();
        outputFile = std::ofstream(log_name);
    }
    int end_flag = 0;

    std::vector<path_point_t> splined_points_size_cutting = get_path(map_file_path, TRACK_POINT_SPLIT_MAX, splined_points_cursor);

    if (splined_points_size_cutting.size() < TRACK_POINT_SPLIT_MAX) {
        splined_points_cursor += splined_points_size_cutting.size();
        end_flag = 1;
    } else {
        splined_points_cursor += TRACK_POINT_SPLIT_HALF;
    }
    // path_point_t origin = {splined_points_size_cutting[0].x, splined_points_size_cutting[0].y, 0,0,0};
    pt_control_state_t init_state = {splined_points_size_cutting[0].x, splined_points_size_cutting[0].y, splined_points_size_cutting[0].yaw, 0, 0};
    tracker.set_path_points(init_state, splined_points_size_cutting);


    while (true) {
        if (tracker.get_remain_point_num() < TRACK_POINT_SPLIT_HALF && (end_flag == 0)) {
            splined_points_size_cutting = get_path(map_file_path, TRACK_POINT_SPLIT_MAX, splined_points_cursor);
            if (splined_points_size_cutting.size() < TRACK_POINT_SPLIT_MAX) { // 남은 개수가 절반보다 남지 않은 경우
                splined_points_cursor = 0;
                end_flag = 1;
            } else {
                splined_points_cursor += TRACK_POINT_SPLIT_HALF;
            }
            tracker.set_path_points(tracker.get_last_updated_state(), splined_points_size_cutting);
        }
        pt_update_result_t update_result = tracker.update(time_sum);
        time_sum += 0.05;
        if (update_result != PT_UPDATE_RESULT_RUNNING) {
            if (update_result == PT_UPDATE_RESULT_NOT_READY) {
                continue;
            }
            printf("finish %d", (int)update_result);
            return 0;
        } else {
            static int debug_count = 0;
            // if (debug_count % 10 == 0) {
                outputFile = std::ofstream(log_name, std::ios::app);

                // 파일이 열렸는지 확인
                if (!outputFile.is_open()) {
                    std::cerr << "can not open." << std::endl;
                    return 1;
                }

                pt_control_state_t current_state = tracker.get_predict_state();
                char debug_string[200];
                int index = tracker.get_front_target_point_index();
                sprintf(debug_string, "%lf,%lf,%lf,%lf,%lf,%lf,,%lf,%lf,%lf",current_state.x,
                                                                    current_state.y,
                                                                    current_state.yaw,
                                                                    current_state.steer,
                                                                    tracker.get_yaw_error(),
                                                                    current_state.v,
                                                                    splined_points_size_cutting[index].x,
                                                                    splined_points_size_cutting[index].y,
                                                                    splined_points_size_cutting[index].yaw);
                outputFile << debug_string << "\n";
                // std::cout << debug_string << std::endl;
                outputFile.close();
            // }
            debug_count++;
        }
    }


    return 0;
}

