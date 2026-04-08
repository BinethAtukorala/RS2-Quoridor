cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------- Packages --------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# -------------------- Executables --------------------
add_executable(control src/control.cpp)
add_executable(pose_test src/pose_test.cpp)
add_executable(get_pose src/get_pose.cpp)
add_executable(gripper src/gripper.cpp)

# -------------------- C++ Standards --------------------
target_compile_features(control PUBLIC c_std_99 cxx_std_17)
target_compile_features(pose_test PUBLIC c_std_99 cxx_std_17)
target_compile_features(get_pose PUBLIC c_std_99 cxx_std_17)
target_compile_features(gripper PUBLIC c_std_99 cxx_std_17)

# -------------------- Dependencies --------------------
ament_target_dependencies(
  control
  rclcpp
  moveit_ros_planning_interface
  std_msgs
  geometry_msgs
)

ament_target_dependencies(
  pose_test
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

ament_target_dependencies(
  get_pose
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

ament_target_dependencies(
  gripper
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# -------------------- Install Targets --------------------
install(TARGETS control
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS pose_test
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS get_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS gripper
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# -------------------- Testing / Lint --------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()