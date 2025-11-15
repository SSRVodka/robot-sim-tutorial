cmake_minimum_required(VERSION 3.8)
project(sys_stat_display)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sys_stat_if REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

set(UI_SRC ui/main_window.ui)

set(MOC_HEADERS src/main_window.h)

set(MAIN_SRC
  src/main_window.cpp
  src/stat_display.cpp)

QT5_WRAP_UI(COMPILED_FORMS ${UI_SRC})
QT5_WRAP_CPP(MOCd_HEADERS ${MOC_HEADERS})

add_executable(${PROJECT_NAME}
  ${MAIN_SRC}
  ${COMPILED_FORMS}
  ${MOCd_HEADERS}
  ${MOC_HEADERS})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_BINARY_DIR})

ament_target_dependencies(${PROJECT_NAME} rclcpp sys_stat_if)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
)

install(TARGETS ${PROJECT_NAME} DESTINATION bin/${PROJECT_NAME})

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
