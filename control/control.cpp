#include <memory>
#include <thread>
#include <vector>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>


double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

std::vector<double> toRadians(const std::vector<double>& degs) {
    std::vector<double> rads;
    for (double d : degs)
        rads.push_back(deg2rad(d));
    return rads;
}

void moveGripper(
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub,
    double width)
{
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {width};
    pub->publish(msg);
}

bool moveToWaypoint(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const std::vector<double> &target,
    const std::string &name,
    rclcpp::Logger logger)
{
    move_group.setJointValueTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_ERROR(logger, "Planning failed for %s", name.c_str());
        return false;
    }

    auto exec_result = move_group.execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Execution failed for %s", name.c_str());
        return false;
    }

    RCLCPP_INFO(logger, "Completed %s", name.c_str());
    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "control",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto logger = node->get_logger();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "ur_onrobot_manipulator");

    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);

    auto gripper_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/finger_width_controller/commands", 10
    );

    std::vector<double> wp1 = toRadians({-70, -110, 20, -140, 90, 0});
    std::vector<double> wp2 = toRadians({-80, -120, 10, -130, 90, 10});
    std::vector<double> wp3 = toRadians({-60, -100, 30, -150, 90, -10});

    moveToWaypoint(move_group, wp1, "Waypoint 1", logger);

    RCLCPP_INFO(logger, "Opening gripper");
    moveGripper(gripper_pub, 0.04);

    moveToWaypoint(move_group, wp2, "Waypoint 2", logger);

    RCLCPP_INFO(logger, "Closing gripper");
    moveGripper(gripper_pub, 0.02);

    moveToWaypoint(move_group, wp3, "Waypoint 3", logger);

    rclcpp::shutdown();
    return 0;
}
