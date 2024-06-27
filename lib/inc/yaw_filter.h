#pragma once

#include <path_manager.h>
#include <qformat.h>
#include <model_matrix.h>
#include <vector>

// std
#include <stddef.h>
#include <stdint.h>

bool yaw_filter_init();

void yaw_filter_set_yaw(float yaw);
float yaw_filter_get_yaw();
float yaw_filter_get_V();
void yaw_filter_set_R(float R_value);
float yaw_filter_predict(float angular_velocity, float updated_time);
bool yaw_filter_valid_gate(ModelMatrix innovation, ModelMatrix H, float sigma);
float yaw_filter_estimate(float yaw, float sigma);

void yaw_filter_set_last_update_time(float update_time);
