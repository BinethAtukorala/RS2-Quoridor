#include <memory>
#include <thread>
#include <vector>
#include <chrono>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>

double deg2rad(double deg) { return deg * M_PI / 180.0; }

std::vector<double> toRadians(const std::vector<double>& degs)
{
    std::vector<double> rads;
    for (double d : degs)
        rads.push_back(deg2rad(d));
    return rads;
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

    // -------------------- GRIPPER COMMAND --------------------
    auto gripper_pub = node->create_publisher<std_msgs::msg::String>(
        "/gripper/command", 10
    );

    bool gripper_done = false;
    auto gripper_sub = node->create_subscription<std_msgs::msg::String>(
        "/gripper/status", 10, [&](std_msgs::msg::String::SharedPtr msg){
            // Update done for any gripper command that finishes
            if(msg->data == "done" || msg->data == "success" || msg->data == "fail" || msg->data == "wrong")
                gripper_done = true;
        }
    );

    auto publishGripperCommand = [&](const std::string &cmd){
        gripper_done = false;
        std_msgs::msg::String msg;
        msg.data = cmd;
        RCLCPP_ERROR(logger, "DEBUG: COMMAND = %s", cmd.c_str());
        gripper_pub->publish(msg);
        RCLCPP_INFO(logger, "Sent gripper command: %s", cmd.c_str());

        // wait for gripper to actually finish the action
        rclcpp::Rate rate(20); // faster check
        while(!gripper_done && rclcpp::ok())
            rate.sleep();
    };

    // // -------------------- WAYPOINTS --------------------
    std::vector<double> wp1 = toRadians({-72.94, -109.89, -9.94, -150.05, 90, 132.49});
    std::vector<double> wp2 = toRadians({-80.41, -123.65, -32.68, -114.91, 89.26, 97.76});
    std::vector<double> wp3 = toRadians({-80.41, -118.73, -60.36, -92.16, 89.28, 97.77});
    std::vector<double> wp4 = toRadians({-80.41, -123.65, -32.68, -114.91, 89.26, 97.76});
    std::vector<double> wp5 = toRadians({-72.94, -109.89, -9.94, -150.05, 90, 132.4});
    std::vector<double> wp6 = toRadians({-69.84, -121.42, -31.66, -117.77, 87.17, 107.72});
    std::vector<double> wp7 = toRadians({-69.84, -115.24, -65.25, -90.35, 87.19, 107.73});
    std::vector<double> wp8 = toRadians({-69.84, -121.42, -31.66, -117.77, 87.17, 107.72});
    // // ===== Declare all waypoints =====
    // std::vector<double> wp1  = toRadians({-51.71, -77.21, -57.57, -132.27, 88.43, 47.16});
    // std::vector<double> wp2  = toRadians({-67.23, -82.90, -66.70, -124.73, 92.55, 27.23});
    // std::vector<double> wp3  = toRadians({-149.04, -106.06, -64.80, -124.96, 122.64, 267.68});
    // std::vector<double> wp4  = toRadians({-169.47, -112.07, -63.04, -122.01, 118.40, 262.45});
    // std::vector<double> wp5  = toRadians({5.15, -118.24, -43.78, -146.88, 69.67, 152.21});
    // std::vector<double> wp6  = toRadians({0, -90, 0, -90, 0, 0});
    // std::vector<double> wp7  = toRadians({2.12, -137.95, -1.93, -171.61, 79.04, 62.98});
    // std::vector<double> wp8  = toRadians({-65.50, -71.93, -79.91, -117.71, 100.82, 44.27});
    // std::vector<double> wp9  = toRadians({-130.91, -115.49, -47.46, -134.52, 122.94, 341.55});
    // std::vector<double> wp10 = toRadians({-153.98, -133.57, -26.99, -150.92, 117.23, 328.10});
    // std::vector<double> wp11 = toRadians({1.41, -122.57, -53.17, -142.15, 78.92, 85.11});
    // std::vector<double> wp12 = toRadians({-61.06, -128.71, -7.91, -154.12, 93.08, 26.51});
    // std::vector<double> wp13 = toRadians({-96.78, -130.48, -4.82, -157.74, 112.64, 12.02});
    // std::vector<double> wp14 = toRadians({-44.04, -133.01, 8.49, -167.27, 85.40, 16.76});
    // std::vector<double> wp15 = toRadians({-90.29, -30.27, -93.90, -122.45, 105.13, 48.85});
    // std::vector<double> wp16 = toRadians({-118.09, -120.95, -29.98, -142.84, 114.54, 15.19});
    // std::vector<double> wp17 = toRadians({-25.53, -114.77, -18.44, -158.34, 81.86, 36.20});
    // std::vector<double> wp18 = toRadians({-125.10, -71.30, -128.24, -57.48, 118.95, 284.37});
    // std::vector<double> wp19 = toRadians({-7.66, -82.62, -91.64, -128.83, 75.47, 56.78});
    // std::vector<double> wp20 = toRadians({-52.52, -92.71, 1.43, -173.62, 84.10, 15.67});

    // // ===== Move to each waypoint =====
    // moveToWaypoint(move_group, wp1,  "Waypoint 1",  logger);
    // moveToWaypoint(move_group, wp2,  "Waypoint 2",  logger);
    // moveToWaypoint(move_group, wp3,  "Waypoint 3",  logger);
    // moveToWaypoint(move_group, wp4,  "Waypoint 4",  logger);
    // moveToWaypoint(move_group, wp5,  "Waypoint 5",  logger);
    // moveToWaypoint(move_group, wp6,  "Waypoint 6",  logger);
    // moveToWaypoint(move_group, wp7,  "Waypoint 7",  logger);
    // moveToWaypoint(move_group, wp8,  "Waypoint 8",  logger);
    // moveToWaypoint(move_group, wp9,  "Waypoint 9",  logger);
    // moveToWaypoint(move_group, wp10, "Waypoint 10", logger);
    // moveToWaypoint(move_group, wp11, "Waypoint 11", logger);
    // moveToWaypoint(move_group, wp12, "Waypoint 12", logger);
    // moveToWaypoint(move_group, wp13, "Waypoint 13", logger);
    // moveToWaypoint(move_group, wp14, "Waypoint 14", logger);
    // moveToWaypoint(move_group, wp15, "Waypoint 15", logger);
    // moveToWaypoint(move_group, wp16, "Waypoint 16", logger);
    // moveToWaypoint(move_group, wp17, "Waypoint 17", logger);
    // moveToWaypoint(move_group, wp18, "Waypoint 18", logger);
    // moveToWaypoint(move_group, wp19, "Waypoint 19", logger);
    // moveToWaypoint(move_group, wp20, "Waypoint 20", logger);

    
    // -------------------- SEQUENCE --------------------
    moveToWaypoint(move_group, wp1, "Waypoint 1", logger);
    moveToWaypoint(move_group, wp2, "Waypoint 2", logger);

    publishGripperCommand("open");          // fully close gripper
    moveToWaypoint(move_group, wp3, "Waypoint 3", logger);

    publishGripperCommand("pickup_pawn");    // pickup wall safely
    moveToWaypoint(move_group, wp4, "Waypoint 4", logger);

    moveToWaypoint(move_group, wp5, "Waypoint 5", logger);
    moveToWaypoint(move_group, wp6, "Waypoint 6", logger);
    moveToWaypoint(move_group, wp7, "Waypoint 7", logger);

    publishGripperCommand("drop_pawn");      // drop wall safely
    moveToWaypoint(move_group, wp8, "Waypoint 8", logger);

    rclcpp::shutdown();
    return 0;
}