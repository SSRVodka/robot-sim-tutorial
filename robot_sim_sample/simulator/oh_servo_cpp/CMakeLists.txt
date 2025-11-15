cmake_minimum_required(VERSION 3.8)
project(oh_servo_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)

set(ROS2_DEPS rclcpp geometry_msgs control_msgs)

set(OH_SERVO_LIBNAME ohservo)
set(OH_SERVO
  src/servo_controller.cpp
)
set(OH_SERVO_DEMO oh_servo_demo)
set(OH_KEYBOARD_SERVO
  src/keyboard_servo.cpp
  src/keyboard_servo_demo.cpp
)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_library(${OH_SERVO_LIBNAME} SHARED ${OH_SERVO})
ament_target_dependencies(${OH_SERVO_LIBNAME} ${ROS2_DEPS})

add_executable(${OH_SERVO_DEMO} ${OH_KEYBOARD_SERVO} ${OH_SERVO})
ament_target_dependencies(${OH_SERVO_DEMO} ${ROS2_DEPS})

install(
  TARGETS ${OH_SERVO_LIBNAME} ${OH_SERVO_DEMO}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
