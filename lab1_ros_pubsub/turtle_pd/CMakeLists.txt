cmake_minimum_required(VERSION 3.8)
project(turtle_pd)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(turtle_pd_if REQUIRED)

set(PROJ_NAME turtle_cont)
set(DEMO_NAME patrol_demo)
add_executable(${PROJ_NAME} src/turtle_controller.cpp)
add_executable(${DEMO_NAME} src/patrol_client.cpp)
ament_target_dependencies(${PROJ_NAME} rclcpp geometry_msgs turtlesim turtle_pd_if)
ament_target_dependencies(${DEMO_NAME} rclcpp turtle_pd_if)

install(TARGETS ${PROJ_NAME} DESTINATION bin/${PROJECT_NAME})
install(TARGETS ${DEMO_NAME} DESTINATION bin/${PROJECT_NAME})

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
