#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<rclcpp::Node>("control");

  auto logger = rclcpp::get_logger("control");
  RCLCPP_INFO(logger, "Starting simple motion example");

  // Create MoveGroupInterface
  moveit::planning_interface::MoveGroupInterface move_group_interface(node, "ur_manipulator");

  // Get current joint values
  std::vector<double> joint_goal = move_group_interface.getCurrentJointValues();

  // Apply a simple movement (rotate base joint slightly)
  joint_goal[0] += 0.1;

  move_group_interface.setJointValueTarget(joint_goal);

  // Plan motion
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(logger, "Planning successful, executing...");
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed");
  }

  // Shutdown
  rclcpp::shutdown();
  return 0;
}
