# Simple Robotics Utils

[![Run Tests](https://github.com/RicoJia/SimpleRoboticsUtils/actions/workflows/test.yml/badge.svg)](https://github.com/RicoJia/SimpleRoboticsUtils/actions/workflows/test.yml)

This repo has common tools for robotics development, in C++ and Python. 

Dependencies:
- `simple_robotics_cpp_utils` 
    - ROS Is NOT a dependency
- `simple_robotics_ros_cpp_utils`
    - ROS IS a dependency. You can either copy it to your workspace, or if you use docker, you can mount this as a volume to avoid copying.

## simple_robotics_python_utils

This is the Python toolkit. It includes:
- `rpi_lidar_a1_visualization.py`, an exportable module that could be used as a Rpi Lidar A1 driver in other projects. 

To install:
`pip install -e .`

## simple_robotics_common_udev_rules

This directory is a collection of udev rules for various USB devices (sensors) that I found useful. They shouldn't cause conflicts and are being actively maintained.

### Usage

1. Just copy the udev rules you need to `/etc/udev/rules.d`
2. Or if you are using a docker container, you can choose to mount this directory to `/etc/udev/rules.d` in your container.
    ```bash
    docker run ... 
    -v simple_robotics_common_udev_rules:/etc/udev/rules.d \
    -v /dev/bus/usb:/dev/bus/usb \
    ...
    ```

### Components

- `99-realsense-libusb.rules` librealsense's udev rules

## Release Workflow

When you want to release a new version, update the version number in your `setup.py` or `pyproject.toml`, then commit and push a new tag to master:

```bash
git tag v1.0.0
git push origin v1.0.0 
```

This will trigger the publish workflow and automatically push the package to PyPI.

- Trigger when a new tag starting with v (e.g., v1.0.0) is pushed to the master branch.
- Build the package using build and twine.
- Publish the package to PyPI.

To contribute, after making changes, one can:

```bash
git tag -d v1.0.0  && git push origin :refs/tags/v1.0.0 && git tag -a v1.0.0 -m "pubsub, io, controllers, and basic handcrafted computer vision components && git push origin v1.0.0" && git push origin v1.0.0
```
