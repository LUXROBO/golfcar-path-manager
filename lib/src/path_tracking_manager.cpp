#include "path_tracking_manager.h"
#include <cmath>
#include <algorithm>
#include <stdio.h>
#include <iostream>

// #include "//ESP_LOg.h"

#define LQR_TAG "LQR_RUNNER_TAG"

const double MAX_STEER = 45.0 * M_PI / 180.0;
const double MAX_SPEED = 10.0 / 3.6;
const double WB = 0.41;                      //앞 뒤 바퀴 사이 거리

path_tracking_manager::path_tracking_manager()
{

}

path_tracking_manager::~path_tracking_manager()
{

}

void path_tracking_manager::add_course(ControlState init_state, std::vector<Point> points)
{
    this->points.insert(end(this->points), begin(points), end(points));

    int goal_index = points.size() - 1;

    this->init_state = init_state;  // init start state
    if (this->init_state.yaw - this->points[0].yaw >= M_PI) {
        this->init_state.yaw -= 2.0 * M_PI;
    } else if (this->init_state.yaw - this->points[0].yaw <= -M_PI) {
        this->init_state.yaw += 2.0 * M_PI;
    }
    this->goal_state = ControlState(this->points[goal_index].x, this->points[goal_index].y, this->points[goal_index].yaw, 0, this->points[goal_index].speed);
    this->t = 0.0;

    this->target_ind = this->calculate_nearest_index(this->init_state, this->points, 0);
    this->smooth_yaw(this->points);
}

void path_tracking_manager::smooth_yaw(std::vector<Point> &points)
{
    for (uint32_t i = 0; i < points.size() - 1; i++)
    {
        double diff_yaw = points[i + 1].yaw - points[i].yaw;

        while (diff_yaw >= M_PI_2) {
            points[i + 1].yaw -= M_PI * 2.0;
            diff_yaw = points[i + 1].yaw - points[i].yaw;
        }

        while (diff_yaw <= -M_PI_2) {
            points[i + 1].yaw += M_PI * 2.0;
            diff_yaw = points[i + 1].yaw - points[i].yaw;
        }
    }
}


ControlState path_tracking_manager::update_state(ControlState state, double accel, double steer_delta, double dt)
{
    if (steer_delta > MAX_STEER) {
        steer_delta = MAX_STEER;
    } else if (steer_delta < -MAX_STEER) {
        steer_delta = -MAX_STEER;
    }
    state.steer = steer_delta;
    // golfcar position, angle update
    state.x = state.x + state.v * std::cos(state.yaw) * dt;
    state.y = state.y + state.v * std::sin(state.yaw) * dt;
    state.yaw = state.yaw + state.v / WB * std::tan(steer_delta) * dt;
    state.v = state.v + accel * dt;

    if (state.v > MAX_SPEED) {
        state.v = MAX_SPEED;
    } else if (state.v < -MAX_SPEED) {
        state.v = -MAX_SPEED;
    }

    return state;
}
