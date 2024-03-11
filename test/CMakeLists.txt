cmake_minimum_required(VERSION 3.0)

project(golfcar_lqr_path_manager_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W0")

file(GLOB SOURCES
    ${PROJECT_SOURCE_DIR}/../lib/src/*cpp
    ${PROJECT_SOURCE_DIR}/test.cpp
)

file(GLOB HEADER
    ${PROJECT_SOURCE_DIR}/../lib/inc
    ${PROJECT_SOURCE_DIR}
)

include_directories(
  ${HEADER}
)

add_executable(${PROJECT_NAME}
    ${SOURCES}
)
