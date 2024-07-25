# This is a ROS-independent C++ project How to build this: mkdir build cd build
# cmake .. make make test  # or directly use 'ctest' for more options sudo make
# install

# Stage 1: specify the project
cmake_minimum_required(VERSION 3.10)
project(simple_robotics_cpp_utils)
set(CMAKE_CXX_STANDARD 20)

# Stage 2: find packages
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

# Stage 3: include directories
include_directories(include ${OpenCV_INCLUDE_DIRS})

# Stage 4: Create Library Target
file(GLOB SOURCES "src/*.cpp")
message("GLOB: ${SOURCES}")
add_library(${PROJECT_NAME} STATIC ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} Threads::Threads
                      Eigen3::Eigen)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         # $<INSTALL_INTERFACE:include>
         ${OpenCV_INCLUDE_DIRS})

# Stage 5: Create Test Targets
file(GLOB TEST_SRC_FILES tests/*.cpp)
add_executable(${PROJECT_NAME}_test ${TEST_SRC_FILES})
target_link_libraries(
  ${PROJECT_NAME}_test
  gtest
  gtest_main
  Threads::Threads
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ${PROJECT_NAME})
# gtest requires the pthread library
add_test(NAME ${PROJECT_NAME}_test COMMAND ${PROJECT_NAME}_test)
enable_testing()

# # Stage 6: Install library install(TARGETS your_library DESTINATION
# /usr/local/lib) install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION lib
# LIBRARY DESTINATION lib RUNTIME DESTINATION bin ) install(DIRECTORY include/
# DESTINATION /usr/local/include)
