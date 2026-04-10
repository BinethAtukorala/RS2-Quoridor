#include <memory>
#include <thread>
#include <vector>
#include <chrono>
#include <cmath>
#include <string>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
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

    if (!success) {
        RCLCPP_ERROR(logger, "Planning failed for %s", name.c_str());
        return false;
    }

    RCLCPP_INFO(logger, "Executing %s", name.c_str());
    auto exec_result = move_group.execute(plan);

    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Execution failed for %s", name.c_str());
        return false;
    }

    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
}

// ================================================================
// MAIN
// ================================================================
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "record_waypoints",
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

    // ================= FILE =================
    std::ofstream file("waypoints_pose.txt");
    if (!file.is_open()) {
        RCLCPP_ERROR(logger, "Failed to open file!");
        return 1;
    }

    // ================= WAYPOINTS =================
    std::vector<std::vector<double>> waypoints = {
        toRadians({0,-90,0,-90,0,0}),
        toRadians({-52.63, -92.84, -14.33, -160.38, 88.81, 41.67}),
        toRadians({-120.8, -110.10, -15.02, -152.63, 116.56, 332.95}),
        toRadians({-140.11, -119.04, -5.55, -156.37, 112.21, 310.43}),
        toRadians({-13.14, -117.39, -4.28, -166.47, 78.92, 75.14}),
        toRadians({-6.99, -121.59, 8.09, -173.47, 77.16, 96.89}),
        toRadians({-71.63, -79.10, -59.38, -130.16, 91.17, 18.91}),
        toRadians({-113.19, -90.35, -48.58, -135.44, 104.09, 348.89}),
        toRadians({-135.57, -109.86, -38.41, -132.82, 111.57, 323.41}),
        toRadians({-21.59, -104.06, -44.06, -140.26, 80.28, 69.90}),
        toRadians({-20.88, -120.29, -17.26, -151.08, 74.78, 97.60}),
        toRadians({-56.42, -61.29, -108.51, -100.98, 87.67, 41.50}),
        toRadians({-123.76, -76.63, -90.47, -103.28, 112.29, 344.91}),
        toRadians({-146.73, -119.02, -42.52, -129.22, 114.51, 310.45}),
        toRadians({-10.10, -94.42, -74.86, -129.40, 75.06, 80.32}),
        toRadians({0.26, -132.75, -37.61, -141.47, 78.37, 99.17}),
        toRadians({-63.40, -51.02, -38.41, -152.44, 84.42, 41.77}),
        toRadians({-2.86, -104.22, -30.47, -155.74, 66.54, 132.64}),
        toRadians({-94.90, -100.98, -13.24, -156.22, 109.28, 43.81}),
        toRadians({-24.38, -112.52, -0.44, -166.88, 83.83, 43.11}),
        toRadians({-90.61, -37.41, -104.41, -107.87, 100.40, 0.45})

        // toRadians({-52.63, -92.84, -14.33, -160.38, 88.81, 41.67}),
        // toRadians({...}),
        // toRadians({...}),
    };

    // ================= LOOP =================
    for (size_t i = 0; i < waypoints.size(); i++)
    {
        std::string name = "Waypoint " + std::to_string(i + 1);

        bool success = moveToWaypoint(move_group, waypoints[i], name, logger);
        if (!success) continue;

        // -------- GET POSE --------
        geometry_msgs::msg::Pose pose;
        try {
            pose = move_group.getCurrentPose().pose;
        }
        catch (...) {
            RCLCPP_ERROR(logger, "Failed to get pose at %s", name.c_str());
            continue;
        }

        // -------- WRITE TO FILE --------
        // file << "Waypoint " << i+1 << "\n";
        // file << "Position: "
        //      << pose.position.x << " "
        //      << pose.position.y << " "
        //      << pose.position.z << "\n";

        // file << "Orientation: "
        //      << pose.orientation.x << " "
        //      << pose.orientation.y << " "
        //      << pose.orientation.z << " "
        //  << pose.orientation.w << "\n\n";

        file << pose.position.x << ","
        << pose.position.y << ","
        << pose.position.z << ","
        << pose.orientation.x << ","
        << pose.orientation.y << ","
        << pose.orientation.z << ","
        << pose.orientation.w << "\n";

        RCLCPP_INFO(logger, "Saved pose for %s", name.c_str());
    }

    file.close();
    RCLCPP_INFO(logger, "All poses saved to waypoints_pose.txt");

    rclcpp::shutdown();
    return 0;
}