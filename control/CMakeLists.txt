cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(quoridor_interfaces REQUIRED)

add_executable(control src/control.cpp)
add_executable(pose_test src/pose_test.cpp)
add_executable(get_pose src/get_pose.cpp)
add_executable(gripper src/gripper.cpp)
add_executable(calibration_move src/calibration_move.cpp)
add_executable(record_waypoints src/record_waypoints.cpp)
add_executable(save_current_state src/save_current_state.cpp)
add_executable(replay_and_save src/replay_and_save.cpp)
add_executable(poses src/poses.cpp)

target_compile_features(control PUBLIC c_std_99 cxx_std_17)
target_compile_features(pose_test PUBLIC c_std_99 cxx_std_17)
target_compile_features(get_pose PUBLIC c_std_99 cxx_std_17)
target_compile_features(gripper PUBLIC c_std_99 cxx_std_17)
target_compile_features(calibration_move PUBLIC c_std_99 cxx_std_17)
target_compile_features(record_waypoints PUBLIC c_std_99 cxx_std_17)
target_compile_features(save_current_state PUBLIC c_std_99 cxx_std_17)
target_compile_features(replay_and_save PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  control
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  shape_msgs
  tf2
  quoridor_interfaces
)

ament_target_dependencies(
  pose_test
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
  tf2
)

ament_target_dependencies(
  get_pose
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  sensor_msgs
)

ament_target_dependencies(
  gripper
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

ament_target_dependencies(
  calibration_move
  rclcpp
  moveit_ros_planning_interface
  std_msgs
  geometry_msgs
)

ament_target_dependencies(
  record_waypoints
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

ament_target_dependencies(
  save_current_state
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

ament_target_dependencies(
  replay_and_save
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

ament_target_dependencies(poses rclcpp rclcpp_action quoridor_interfaces)

install(TARGETS
  control
  pose_test
  get_pose
  gripper
  calibration_move
  record_waypoints
  save_current_state
  replay_and_save
  poses
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/camera_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()