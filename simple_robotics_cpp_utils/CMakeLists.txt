# This is a ROS-independent C++ project How to build this: mkdir build cd build
# cmake .. make make test  # or directly use 'ctest' for more options sudo make
# install

cmake_minimum_required(VERSION 3.10)
project(simple_robotics_cpp_utils)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
set(CMAKE_CXX_STANDARD 17)

include_directories(include)

# Install library install(TARGETS your_library DESTINATION /usr/local/lib)
install(DIRECTORY include/ DESTINATION /usr/local/include)

file(GLOB TEST_SRC_FILES tests/*.cpp)

# Tests
add_executable(${PROJECT_NAME}_test ${TEST_SRC_FILES})

target_link_libraries(${PROJECT_NAME}_test gtest gtest_main Threads::Threads
                      Eigen3::Eigen)
# gtest requires the pthread library
find_package(Threads REQUIRED)
add_test(NAME ${PROJECT_NAME}_test COMMAND ${PROJECT_NAME}_test)
enable_testing()
