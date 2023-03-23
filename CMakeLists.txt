cmake_minimum_required(VERSION 3.0)

project(golfcar_lqr_path_manager)

set(CMAKE_CXX_STANDARD 11)

file(GLOB SOURCES
    ${PROJECT_SOURCE_DIR}/test.cpp
    ${PROJECT_SOURCE_DIR}/src/*.cpp
)

file(GLOB HEADER 
    ${PROJECT_SOURCE_DIR}/inc
)

include_directories(
  ${HEADER}
)

add_executable(${PROJECT_NAME}
    ${SOURCES}
)


# target_link_libraries(${PROJECT_NAME} MyLibrary)