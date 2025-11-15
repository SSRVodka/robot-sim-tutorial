#!/bin/bash

ros2 pkg create demo_cpp_node --build-type ament_cmake --license Apache-2.0 --dependencies rclcpp

ros2 pkg create demo_py_node --build-type ament_python --license Apache-2.0 --dependencies rclpy
