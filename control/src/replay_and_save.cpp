#include <memory>
#include <thread>
#include <chrono>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "replay_and_save",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto logger = node->get_logger();

    // Executor thread
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "ur_onrobot_manipulator");

    rclcpp::sleep_for(std::chrono::seconds(2));

    // ================= INPUT FILE =================
    std::ifstream infile("calib_pose.txt");
    if (!infile.is_open()) {
        RCLCPP_ERROR(logger, "Failed to open input file!");
        return 1;
    }

    // ================= OUTPUT FILE =================
    std::ofstream outfile("calib_pose_2.txt", std::ios::app);
    if (!outfile.is_open()) {
        RCLCPP_ERROR(logger, "Failed to open output file!");
        return 1;
    }

    std::string line;
    int waypoint_id = 0;

    while (std::getline(infile, line))
    {
        waypoint_id++;
        std::stringstream ss(line);
        std::vector<double> joints;

        std::string value;
        int count = 0;

        // ================= PARSE FIRST 6 VALUES =================
        while (std::getline(ss, value, ',') && count < 6)
        {
            joints.push_back(std::stod(value));
            count++;
        }

        if (joints.size() != 6) {
            RCLCPP_WARN(logger, "Skipping invalid line %d", waypoint_id);
            continue;
        }

        RCLCPP_INFO(logger, "Moving to waypoint %d...", waypoint_id);

        // ================= MOVE ROBOT =================
        move_group.setJointValueTarget(joints);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
            RCLCPP_WARN(logger, "Planning failed at waypoint %d", waypoint_id);
            continue;
        }

        move_group.execute(plan);

        // small delay to stabilize
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // ================= GET POSE (tool0 = no gripper) =================
        geometry_msgs::msg::Pose pose;
        try {
            pose = move_group.getCurrentPose("wrist_3_link").pose;
        }
        catch (...) {
            RCLCPP_ERROR(logger, "Failed to get pose at waypoint %d", waypoint_id);
            continue;
        }

        // ================= SAVE =================
        // for (size_t i = 0; i < joints.size(); i++)
        // {
        //     outfile << joints[i] << ",";
        // }

        outfile << pose.position.x << ","
                << pose.position.y << ","
                << pose.position.z << ","
                << pose.orientation.x << ","
                << pose.orientation.y << ","
                << pose.orientation.z << ","
                << pose.orientation.w << "\n";

        RCLCPP_INFO(logger, "Saved waypoint %d", waypoint_id);
    }

    infile.close();
    outfile.close();

    RCLCPP_INFO(logger, "All waypoints processed!");

    rclcpp::shutdown();
    return 0;
}