#include <memory>
#include <thread>
#include <chrono>
#include <fstream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "save_current_state",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto logger = node->get_logger();

    // Spin executor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "ur_onrobot_manipulator");

    rclcpp::sleep_for(std::chrono::seconds(2));

    // ================= GET JOINTS =================
    std::vector<double> joints = move_group.getCurrentJointValues();

    // ================= GET POSE =================
    geometry_msgs::msg::Pose pose;
    try {
        pose = move_group.getCurrentPose("tool0").pose;
    }
    catch (...) {
        RCLCPP_ERROR(logger, "Failed to get pose");
        return 1;
    }

    // ================= FILE (APPEND MODE) =================
    std::ofstream file("quoridor_poses.txt", std::ios::app);
    if (!file.is_open()) {
        RCLCPP_ERROR(logger, "Failed to open file!");
        return 1;
    }

    // ================= WRITE =================
    // Format:
    // j1,j2,j3,j4,j5,j6,x,y,z,qx,qy,qz,qw

    for (size_t i = 0; i < joints.size(); i++)
    {
        file << joints[i] << ",";
    }

    file << pose.position.x << ","
         << pose.position.y << ","
         << pose.position.z << ","
         << pose.orientation.x << ","
         << pose.orientation.y << ","
         << pose.orientation.z << ","
         << pose.orientation.w << "\n";

    file.close();

    RCLCPP_INFO(logger, "Saved current robot state to file");

    rclcpp::shutdown();
    return 0;
}