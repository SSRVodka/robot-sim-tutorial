cmake_minimum_required(VERSION 3.12)

project(demo_cpp_node)

add_executable(${PROJECT_NAME} node_demo.cpp)

find_package(rclcpp REQUIRED)

target_include_directories(${PROJECT_NAME} PUBLIC ${rclcpp_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})

