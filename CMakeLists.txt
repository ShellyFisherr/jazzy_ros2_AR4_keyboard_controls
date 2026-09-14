cmake_minimum_required(VERSION 3.8)
project(annin_ar4_keyboard_controll)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  moveit_msgs
  sensor_msgs
  )

add_executable(keyboard_input_reader src/keyboard_input_reader.cpp)
ament_target_dependencies(keyboard_input_reader ${dependencies})

add_executable(movement_translator src/movement_translator.cpp)
ament_target_dependencies(movement_translator ${dependencies})

add_executable(moveit_position_reader src/moveit_position_reader.cpp)
ament_target_dependencies(moveit_position_reader ${dependencies})

add_executable(continuous_movement src/continuous_movement.cpp)
ament_target_dependencies(continuous_movement ${dependencies})

add_executable(dls_reduced_controller src/DLS_reduced_controller.cpp)
ament_target_dependencies(dls_reduced_controller ${dependencies})

install(
  TARGETS 
  keyboard_input_reader
  movement_translator 
  moveit_position_reader
  continuous_movement
  dls_reduced_controller
  DESTINATION lib/${PROJECT_NAME}
)

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
