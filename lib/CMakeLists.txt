cmake_minimum_required (VERSION 3.0)

project (golfcar_path_manager)

# set (${PROJECT_NAME}_VERSION_MAJOR 1)
# set (${PROJECT_NAME}_VERSION_MINOR 0)
# set (${PROJECT_NAME}_VERSION_PATCH 0)

set (CMAKE_CXX_STANDARD 11)

include_directories (
    ${PROJECT_SOURCE_DIR}/inc
)

file (GLOB SOURCES
    # src/cubic_spline_planner.cpp
    # src/model_matrix.cpp
    src/qformat_c.c
    src/*.cpp
)

add_library(${PROJECT_NAME} STATIC ${SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../output
)
