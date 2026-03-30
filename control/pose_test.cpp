#include <memory>
#include <chrono>
#include <cmath>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
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

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("check_pose_node");
    auto logger = node->get_logger();

    node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    moveit::planning_interface::MoveGroupInterface move_group(node, "ur_onrobot_manipulator");

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

    RCLCPP_INFO(logger, "Target (mm): X=%.2f Y=%.2f Z=%.2f", x_mm, y_mm, z_mm);
    RCLCPP_INFO(logger, "Target RPY (rad): R=%.3f P=%.3f Y=%.3f", roll, pitch, yaw);

    auto current_pose = move_group.getCurrentPose().pose;

    double cx = current_pose.position.x * 1000.0;
    double cy = current_pose.position.y * 1000.0;
    double cz = current_pose.position.z * 1000.0;

    double cr, cp, cyaw;
    quaternionToRPY(current_pose.orientation, cr, cp, cyaw);

    RCLCPP_INFO(logger, "Current (mm): X=%.2f Y=%.2f Z=%.2f", cx, cy, cz);
    RCLCPP_INFO(logger, "Current RPY (rad): R=%.3f P=%.3f Y=%.3f", cr, cp, cyaw);

    double dx = current_pose.position.x - target_pose.position.x;
    double dy = current_pose.position.y - target_pose.position.y;
    double dz = current_pose.position.z - target_pose.position.z;

    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    RCLCPP_INFO(logger, "Distance to target: %.4f m (%.2f mm)", dist, dist * 1000.0);
    if (dist < 0.03)
        RCLCPP_INFO(logger, "PASS: Within 3 cm threshold!");
    else
        RCLCPP_WARN(logger, "FAIL: Outside threshold!");

    rclcpp::shutdown();
    return 0;
}