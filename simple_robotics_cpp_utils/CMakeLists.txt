cmake_minimum_required(VERSION 3.10)
project(simple_robotics_cpp_utils)

set(CMAKE_CXX_STANDARD 17)
include_directories(include)

# 
add_executable(${PROJECT_NAME}_test 
    tests/test_math_utils.cpp
    # TODO: add more source files here
    )
target_link_libraries(${PROJECT_NAME}_test gtest gtest_main)
# gtest requires the pthread library
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME}_test Threads::Threads)