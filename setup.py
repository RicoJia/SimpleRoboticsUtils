from setuptools import setup, find_packages

setup(
    name="simple_robotics_python_utils",
    version="0.2.0",
    packages=find_packages(
        include=["simple_robotics_python_utils", "simple_robotics_python_utils.*"]
    ),
    install_requires=[
        "posix_ipc",
    ],
)
