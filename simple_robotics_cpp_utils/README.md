# Simple Robotics Cpp Utils

- This is a static cpp libary: despite increased binary size and compile time, static is great for version deployment. Please note, it's **C++20 compatible**
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
    ```
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

### Loguru Logger

[loguru](https://github.com/emilk/loguru) is a great light weight logger. [Its official documentation is here](https://emilk.github.io/loguru/index.html). Specifically, I am happy with:

1. It supports stream logging e.g., `LOG_S(INFO) << "Some float: " << std::setfill('0') << std::setw(5) << std::setprecision(3) << number;`
1. Its supports color printing for `WARNING`, and `ERROR`. (This is suppressed by gtest in testing though.)
1. Its ability to write to a log file.
1. The printing of stack trace when the system crashes!! It's a very helpful feature
