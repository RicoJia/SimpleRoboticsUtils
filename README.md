# Simple Robotics Utils

This repo has common tools for robotics development, in C++ and Python. 

Dependencies:
- C++:
    - ROS Is NOT a dependency

### simple_robotics_python_utils

This is the Python toolkit. It includes:
- `rpi_lidar_a1_visualization.py`, an exportable module that could be used as a Rpi Lidar A1 driver in other projects. 

To install: 
`pip install -e .`

### simple_robotics_common_udev_rules

This directory is a collection of udev rules for various USB devices (sensors) that I found useful. They shouldn't cause conflicts and are being actively maintained.
