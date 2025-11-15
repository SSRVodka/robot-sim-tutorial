cmake_minimum_required(VERSION 3.8)
project(servo2c)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb")

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(moveit_task_constructor_core REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

set(AMENT_DEPS
  rclcpp
  rclcpp_components
  control_msgs
  geometry_msgs
  sensor_msgs
  moveit_msgs
  std_srvs
)

set(MTC_DEPS
  rclcpp
  moveit_task_constructor_core
  tf2_geometry_msgs
  tf2_eigen
)

set(JOY_PLUGIN_NAME joy2servo2p)
set(MTC_DEMO_NAME mtc_node)

add_executable(${MTC_DEMO_NAME} src/mtc_node.cpp)
target_include_directories(${MTC_DEMO_NAME} PRIVATE include)
ament_target_dependencies(${MTC_DEMO_NAME} ${MTC_DEPS})

add_library(${JOY_PLUGIN_NAME} SHARED src/joystick_servo_controller.cpp)
target_include_directories(${JOY_PLUGIN_NAME} PRIVATE include)
ament_target_dependencies(${JOY_PLUGIN_NAME} ${AMENT_DEPS})

rclcpp_components_register_node(
    ${JOY_PLUGIN_NAME}
    PLUGIN "servo2c_plugins::Joy2ServoController"
    EXECUTABLE joy2servo_controller
)

ament_export_targets(export_${JOY_PLUGIN_NAME})
install(TARGETS ${JOY_PLUGIN_NAME}
    EXPORT export_${JOY_PLUGIN_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS ${MTC_DEMO_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

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
