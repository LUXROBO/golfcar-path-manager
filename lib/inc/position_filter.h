#pragma once

#include <path_manager.h>
#include <qformat.h>
#include <model_matrix.h>
#include <vector>

// std
#include <stddef.h>
#include <stdint.h>

typedef struct position_filter_z_format_
{
    float gps_v;
    float yaw_rate;
    float gps_yaw;
    double gps_x;
    double gps_y;
} position_filter_z_format_t;


bool position_filter_init();
bool position_filter_is_init();
void position_filter_set_position(pt_control_state_t position);
pt_control_state_t position_filter_get_position();
void position_filter_set_xy(pt_control_state_t position);
void position_filter_set_yaw(float yaw);
pt_control_state_t position_filter_get_state();
ModelMatrix position_filter_get_predict_x();
bool position_filter_is_init_xy();
// ModelMatrix state_equation_jacobi(ModelMatrix x0, ModelMatrix input);
pt_control_state_t position_filter_predict_xy(float v, float steer, float dt);
pt_control_state_t position_filter_estimate_xy_with_gps(position_filter_z_format_t gps_pos, int quality);
// pt_control_state_t estimate(position_filter_z_format_t estimate_value);
bool position_filter_valid_gate(ModelMatrix z, ModelMatrix x, ModelMatrix H, float sigma);