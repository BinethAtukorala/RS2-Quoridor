#include <memory>
#include <thread>
#include <vector>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

std::vector<double> toRadians(const std::vector<double>& degs)
{
    std::vector<double> rads;
    for (double d : degs)
        rads.push_back(deg2rad(d));
    return rads;
}

void moveGripperSmooth(
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub,
    double &current_width,
    double target_width,
    double step=0.002,
    int delay_ms=20)
{
    std_msgs::msg::Float64MultiArray msg;

    if (current_width < target_width)
    {
        for (double w = current_width; w <= target_width; w += step)
        {
            msg.data = {w};
            pub->publish(msg);
            rclcpp::sleep_for(std::chrono::milliseconds(delay_ms));
        }
    }
    else
    {
        for (double w = current_width; w >= target_width; w -= step)
        {
            msg.data = {w};
            pub->publish(msg);
            rclcpp::sleep_for(std::chrono::milliseconds(delay_ms));
        }
    }

    msg.data = {target_width};
    pub->publish(msg);
    
    current_width = target_width;
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

    if (success) {
        RCLCPP_INFO(logger, "Executing %s", name.c_str());

        auto exec_result = move_group.execute(plan);
        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(logger, "Execution failed for %s", name.c_str());
            return false;
        }
    } else {
        RCLCPP_ERROR(logger, "Planning failed for %s", name.c_str());
        return false;
    }

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

    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);

    rclcpp::sleep_for(std::chrono::seconds(2));

    auto pose = move_group.getCurrentPose();
    RCLCPP_INFO(logger, "Start Z: %f", pose.pose.position.z);

    auto gripper_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/finger_width_controller/commands", 10
    );

    double current_width = 0.0;

    std::vector<double> wp1 = toRadians({-72.94, -109.89, -9.94, -150.05, 90, 132.49});
    std::vector<double> wp2 = toRadians({-80.41, -123.65, -32.68, -114.91, 89.26, 97.76});
    std::vector<double> wp3 = toRadians({-80.41, -118.73, -60.36, -92.16, 89.28, 97.77});
    std::vector<double> wp4 = toRadians({-80.41, -123.65, -32.68, -114.91, 89.26, 97.76});
    std::vector<double> wp5 = toRadians({-72.94, -109.89, -9.94, -150.05, 90, 132.4});
    std::vector<double> wp6 = toRadians({-69.84, -121.42, -31.66, -117.77, 87.17, 107.72});
    std::vector<double> wp7 = toRadians({-69.84, -115.24, -65.25, -90.35, 87.19, 107.73});
    std::vector<double> wp8 = toRadians({-69.84, -121.42, -31.66, -117.77, 87.17, 107.72});

    moveToWaypoint(move_group, wp1, "Waypoint 1", logger);
    moveToWaypoint(move_group, wp2, "Waypoint 2", logger);

    RCLCPP_INFO(logger, "Opening gripper slightly");
    moveGripperSmooth(gripper_pub, current_width, 0.04); // target width only
    rclcpp::sleep_for(std::chrono::seconds(2));

    moveToWaypoint(move_group, wp3, "Waypoint 3", logger);

    RCLCPP_INFO(logger, "Closing gripper");
    moveGripperSmooth(gripper_pub, current_width, 0.022); // target width only
    rclcpp::sleep_for(std::chrono::seconds(2));

    moveToWaypoint(move_group, wp4, "Waypoint 4", logger);
    moveToWaypoint(move_group, wp5, "Waypoint 5", logger);
    moveToWaypoint(move_group, wp6, "Waypoint 6", logger);

    // RCLCPP_INFO(logger, "Opening gripper slightly");
    // moveGripperSmooth(gripper_pub, current_width, 0.04);
    // rclcpp::sleep_for(std::chrono::seconds(2));

    moveToWaypoint(move_group, wp7, "Waypoint 7", logger);

    RCLCPP_INFO(logger, "Opening gripper slightly");
    moveGripperSmooth(gripper_pub, current_width, 0.04);
    rclcpp::sleep_for(std::chrono::seconds(2));
    moveToWaypoint(move_group, wp8, "Waypoint 8", logger);

    rclcpp::shutdown();
    return 0;
}
