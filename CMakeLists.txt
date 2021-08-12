cmake_minimum_required(VERSION 3.16)

project(IEEE_TAES)

find_package(Eigen3 REQUIRED)

add_executable(adaptive main.cpp rungekutta.cpp)

target_link_libraries(adaptive Eigen3::Eigen)