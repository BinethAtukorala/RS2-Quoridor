#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create a node
  auto node = std::make_shared<rclcpp::Node>(
    "control",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto logger = rclcpp::get_logger("control");
  RCLCPP_INFO(logger, "Running (URSim test)");

  // Executor for state updates
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // MoveGroupInterface
  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group_interface(node, "ur_manipulator");

  // Slow movement for easier observation in URSim
  move_group_interface.setMaxVelocityScalingFactor(0.3);
  move_group_interface.setMaxAccelerationScalingFactor(0.3);

  // Print current pose (basic debugging)
  auto current_pose = move_group_interface.getCurrentPose();
  RCLCPP_INFO(logger, "Current Z position: %f", current_pose.pose.position.z);

  // Get current joint values
  std::vector<double> joint_goal = move_group_interface.getCurrentJointValues();

  // Small test motion
  joint_goal[0] += 0.1;

  move_group_interface.setJointValueTarget(joint_goal);

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group_interface.plan(plan));

  if (success) {
    RCLCPP_INFO(logger, "URSim plan successful, executing...");
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown
  rclcpp::shutdown();
  spinner.join();

  return 0;
}
