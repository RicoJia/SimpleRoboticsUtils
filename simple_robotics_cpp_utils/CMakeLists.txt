# This is a ROS-independent C++ project How to build this: mkdir build cd build
# cmake .. make make test  # or directly use 'ctest' for more options sudo make
# install

cmake_minimum_required(VERSION 3.10)
project(simple_robotics_cpp_utils)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

# Install library install(TARGETS your_library DESTINATION /usr/local/lib)
install(DIRECTORY include/ DESTINATION /usr/local/include)

# Tests
add_executable(
  ${PROJECT_NAME}_test tests/test_math_utils.cpp # TODO: add more source files
                                                 # here
)
target_link_libraries(${PROJECT_NAME}_test gtest gtest_main Threads::Threads)
# gtest requires the pthread library
find_package(Threads REQUIRED)
add_test(NAME ${PROJECT_NAME}_test COMMAND ${PROJECT_NAME}_test)
enable_testing()
