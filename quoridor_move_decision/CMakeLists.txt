cmake_minimum_required(VERSION 3.8)
project(quoridor_move_decision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(move_decision
  src/move_decision_node.cpp
  src/board.cpp
)
target_include_directories(move_decision PRIVATE include)
ament_target_dependencies(move_decision rclcpp std_msgs)
target_link_libraries(move_decision nlohmann_json::nlohmann_json)

install(TARGETS move_decision
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
