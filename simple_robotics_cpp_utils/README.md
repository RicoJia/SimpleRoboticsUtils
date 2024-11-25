# Simple Robotics Cpp Utils

This module is a C++ Library for common robotics Uses. It's **ROS-independent**!! 🚀

This is a static cpp libary. Despite increased binary size and compile time, we made it static so linking becomes dirt easy. 

Some things to note:
    - it's **C++20 compatible**
    - Its dependencies such as `OpenCV` are by themselves large, but only the necessary bits are actually copied into this binary.
    - It has a gtest framework, see how things are set up there.

## Usage

- To install,  installed to a Then, 

```bash
./build.sh 
```
    - Without any any input args, it will be installed to `/usr/local/bin` so it's available system wide.
    - If an environment variable `CUSTOM_INSTALL_PATH` is defined, it will be installed there

- To include in your project, in CMakeLists.txt, you can:

```cmake
set(CMAKE_PREFIX_PATH "$ENV{CUSTOM_INSTALL_PATH}" ${CMAKE_PREFIX_PATH})
if(NOT DEFINED ENV{CUSTOM_INSTALL_PATH})
  message(FATAL_ERROR "CUSTOM_INSTALL_PATH environment variable is not set.")
endif()
include_directories(
  $ENV{CUSTOM_INSTALL_PATH}/include
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



