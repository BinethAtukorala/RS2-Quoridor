#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

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
  RCLCPP_INFO(logger, "Running");

  // Start executor so MoveIt can receive robot state updates
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // Create MoveGroupInterface for UR3e manipulator
  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group_interface(node, "ur_manipulator");

  // Add a ground collision object below the robot
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit_msgs::msg::CollisionObject ground;
  ground.id = "ground";
  ground.header.frame_id = "world"; // or "base_link" depending on UR3e setup

  // Create a box primitive for the ground
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {2.0, 2.0, 0.1}; // 2m x 2m x 0.1m thick
  ground.primitives.push_back(box);

  // Position the box just below the robot base
  geometry_msgs::msg::Pose ground_pose;
  ground_pose.orientation.w = 1.0;
  ground_pose.position.x = 0.0;
  ground_pose.position.y = 0.0;
  ground_pose.position.z = -0.05; // half thickness below z=0
  ground.primitive_poses.push_back(ground_pose);

  ground.operation = ground.ADD;
  planning_scene_interface.applyCollisionObject(ground);
  RCLCPP_INFO(logger, "Added ground collision object");

  // Print current end-effector Z position
  auto current_pose = move_group_interface.getCurrentPose();
  RCLCPP_INFO(logger, "Current Z position: %f", current_pose.pose.position.z);

  // // Safe home joint pose
  // std::vector<double> joint_goal = {
  //   0.0,        // base
  //   -1.57,      // shoulder
  //   1.57,       // elbow
  //   -1.57,      // wrist1
  //   -1.57,      // wrist2
  //   0.0         // wrist3
  // };
  // move_group_interface.setJointValueTarget(joint_goal);

  // Get current joint values
  std::vector<double> joint_goal = move_group_interface.getCurrentJointValues();

  // Small safe movement: rotate base joint slightly (+10 degrees)
  joint_goal[0] += 0.17;  // ~10 degrees in radians

  move_group_interface.setJointValueTarget(joint_goal);

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group_interface.plan(plan));

  if (success) {
    RCLCPP_INFO(logger, "Plan successful, executing...");
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS 2
  rclcpp::shutdown();
  spinner.join();

  return 0;
}
