# Simple Robotics Utils

This repo has common tools for robotics development, in C++ and Python. 

Dependencies:
- `simple_robotics_cpp_utils` 
    - ROS Is NOT a dependency
- `simple_robotics_ros_cpp_utils`
    - ROS IS a dependency. You can either copy it to your workspace, or if you use docker, you can mount this as a volume to avoid copying.

### simple_robotics_python_utils

This is the Python toolkit. It includes:
- `rpi_lidar_a1_visualization.py`, an exportable module that could be used as a Rpi Lidar A1 driver in other projects. 

To install: 
`pip install -e .`
