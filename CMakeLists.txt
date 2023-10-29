cmake_minimum_required (VERSION 3.0)

SET(CMAKE_COLOR_MAKEFILE ON)

# add_subdirectory(${PROJECT_SOURCE_DIR}/lib)
add_subdirectory(${PROJECT_SOURCE_DIR}/test)

# set (TARGET golfcar_path_manager)

# set (CMAKE_CXX_STANDARD 11)

# file (GLOB SOURCES
#     ${CMAKE_CURRENT_SOURCE_DIR}lib/src/*.cpp
# )
# set (SOURCE_TEST
#     lib/src/cubic_spline_planner.cpp
#     lib/src/lqr_steer_control.cpp
#     lib/src/model_matrix.cpp
#     lib/src/pid_control.cpp
# )

# idf_component_register(
#     SRCS
#         ${SOURCE_TEST}
#     INCLUDE_DIRS
#         ${CMAKE_CURRENT_SOURCE_DIR}/lib/inc

#     REQUIRES mbedtls)
