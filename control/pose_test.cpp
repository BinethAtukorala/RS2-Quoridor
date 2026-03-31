#include <memory>
#include <thread>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

geometry_msgs::msg::Quaternion rpyToQuaternion(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();
    return q_msg;
}

void quaternionToRPY(const geometry_msgs::msg::Quaternion &q_msg,
                     double &roll, double &pitch, double &yaw)
{
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
}

bool waitForRobotState(moveit::planning_interface::MoveGroupInterface &move_group,
                       rclcpp::Logger logger)
{
    for (int i = 0; i < 10; ++i)
    {
        if (move_group.getCurrentState(1.0))
            return true;

        RCLCPP_WARN(logger, "Waiting for robot state...");
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
    return false;
}

bool moveToPose(moveit::planning_interface::MoveGroupInterface &move_group,
                const geometry_msgs::msg::Pose &target_pose,
                rclcpp::Logger logger)
{
    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(logger, "Executing move...");
        if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(logger, "Execution failed!");
            return false;
        }
    }
    else
    {
        RCLCPP_ERROR(logger, "Planning failed!");
        return false;
    }

    return true;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("move_with_collision_node");

    node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    auto logger = node->get_logger();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "ur_onrobot_manipulator");

    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit_msgs::msg::CollisionObject ground;
    ground.id = "ground";
    ground.header.frame_id = "base_link"; 

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {2.0, 2.0, 0.1};

    geometry_msgs::msg::Pose ground_pose;
    ground_pose.orientation.w = 1.0;
    ground_pose.position.x = 0.0;
    ground_pose.position.y = 0.0;
    ground_pose.position.z = -0.05; 

    ground.primitives.push_back(box);
    ground.primitive_poses.push_back(ground_pose);
    ground.operation = ground.ADD;

    planning_scene_interface.applyCollisionObject(ground);
    RCLCPP_INFO(logger, "Ground collision added");

    rclcpp::sleep_for(std::chrono::seconds(1));

    if (!waitForRobotState(move_group, logger))
    {
        RCLCPP_ERROR(logger, "No robot state!");
        rclcpp::shutdown();
        return 1;
    }

    double x_mm = -0.33;
    double y_mm = 441.15;
    double z_mm = 693.95;
    double roll = 1.569;
    double pitch = -1.569;
    double yaw = -3.138;

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x_mm / 1000.0;
    target_pose.position.y = y_mm / 1000.0;
    target_pose.position.z = z_mm / 1000.0;
    target_pose.orientation = rpyToQuaternion(roll, pitch, yaw);

    RCLCPP_INFO(logger, "----- TARGET -----");
    RCLCPP_INFO(logger, "Pos (mm): X=%.2f Y=%.2f Z=%.2f", x_mm, y_mm, z_mm);
    RCLCPP_INFO(logger, "RPY (rad): R=%.3f P=%.3f Y=%.3f", roll, pitch, yaw);

    if (!moveToPose(move_group, target_pose, logger))
    {
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::sleep_for(std::chrono::seconds(1));

    auto current_pose = move_group.getCurrentPose().pose;

    double cx = current_pose.position.x * 1000.0;
    double cy = current_pose.position.y * 1000.0;
    double cz = current_pose.position.z * 1000.0;

    double cr, cp, cyaw;
    quaternionToRPY(current_pose.orientation, cr, cp, cyaw);

    RCLCPP_INFO(logger, "----- ACTUAL -----");
    RCLCPP_INFO(logger, "Pos (mm): X=%.2f Y=%.2f Z=%.2f", cx, cy, cz);
    RCLCPP_INFO(logger, "RPY (rad): R=%.3f P=%.3f Y=%.3f", cr, cp, cyaw);

    double dx = current_pose.position.x - target_pose.position.x;
    double dy = current_pose.position.y - target_pose.position.y;
    double dz = current_pose.position.z - target_pose.position.z;

    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    RCLCPP_INFO(logger, "----- RESULT -----");
    RCLCPP_INFO(logger, "Error: %.4f m (%.2f mm)", dist, dist * 1000.0);

    if (dist < 0.03)
        RCLCPP_INFO(logger, "PASS");
    else
        RCLCPP_WARN(logger, "FAIL");

    rclcpp::shutdown();
    return 0;
}