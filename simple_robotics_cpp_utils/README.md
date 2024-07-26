# Simple Robotics Cpp Utils

This module is a C++ Library for common robotics Uses. It's **ROS-independent**!! 🚀

This is a static cpp libary. Despite increased binary size and compile time, we made it static so linking becomes dirt easy. 

Some things to note:
    - it's **C++20 compatible**
    - Its dependencies such as `OpenCV` are by themselves large, but only the necessary bits are actually copied into this binary.
    - It has a gtest framework, see how things are set up there.

## Usage

- Vanilla Usage

```bash
mkdir build
cd build
cmake ..
make -j7 
```

- Or, To install it to `/usr/local/bin` so it's available system wide, 

```bash
./build.sh 
```

- To include in your project, in CMakeLists.txt, you can:

```cmake
set(SIMPLE_ROBOTICS_CPP_UTILS_DIR <PATH_TO_THIS_DIR>)
add_subdirectory( ${SIMPLE_ROBOTICS_CPP_UTILS_DIR}
${CMAKE_BINARY_DIR}/simple_robotics_cpp_utils)
include_directories(
    ${SIMPLE_ROBOTICS_CPP_UTILS_DIR}/include
)
target_link_libraries(<YOUR_EXECUTABLE>
    simple_robotics_cpp_utils
)
```

## Components

- BagParser: a tool to step over specified bag messages
- cv_utils: misc open CV utilities, such as displaying images, converting pixel to camera canonical frames, etc.
- io_utils: misc utils such as colored-printing, getting key presses. etc.
- thread_pool: a cpp thread pool adopted from https://github.com/progschj/ThreadPool
- "loguru": [loguru](https://github.com/emilk/loguru) is a great light weight logger. [Its official documentation is here](https://emilk.github.io/loguru/index.html). Specifically, I am happy with:
    1. It supports stream logging e.g., `LOG_S(INFO) << "Some float: " << std::setfill('0') << std::setw(5) << std::setprecision(3) << number;`
    1. Its supports color printing for `WARNING`, and `ERROR`. (This is suppressed by gtest in testing though.)
    1. Its ability to write to a log file.
    1. The printing of stack trace when the system crashes!! It's a very helpful feature



