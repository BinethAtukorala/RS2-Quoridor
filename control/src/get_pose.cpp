#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <map>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>

geometry_msgs::msg::Pose eigenToMsgPose(const Eigen::Isometry3d &pose)
{
    geometry_msgs::msg::Pose msg;
    msg.position.x = pose.translation().x();
    msg.position.y = pose.translation().y();
    msg.position.z = pose.translation().z();

    Eigen::Quaterniond q(pose.rotation());
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();

    return msg;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("get_pose");

    if (argc < 2) {
        RCLCPP_ERROR(node->get_logger(), "Which mode? 1 = MoveIt, 2 = Real Robot");
        return 1;
    }

    int mode = std::atoi(argv[1]);
    if(mode != 1 && mode != 2){
        RCLCPP_ERROR(node->get_logger(), "Invalid mode. Use 1 = MoveIt, 2 = Real Robot");
        return 1;
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "ur_onrobot_manipulator");

    RCLCPP_INFO(node->get_logger(), "Planning frame - %s", move_group.getPlanningFrame().c_str());
    rclcpp::sleep_for(std::chrono::seconds(2));

    geometry_msgs::msg::Pose current_pose;
    bool got_pose = false;

    if(mode == 1){
        RCLCPP_INFO(node->get_logger(), "Mode 1 - MoveIt Pose");

        rclcpp::Rate rate(10);
        for(int i=0; i<50; ++i)
        {
            try
            {
                current_pose = move_group.getCurrentPose().pose;
                got_pose = true;
                break;
            }
            catch(const std::runtime_error &e)
            {
                RCLCPP_WARN(node->get_logger(), "Waiting for MoveIt robot state...");
            }
            rate.sleep();
        }
    }
    else if(mode == 2){
        RCLCPP_INFO(node->get_logger(), "Mode 2 - Real Robot Pose");

        robot_model_loader::RobotModelLoader robot_model_loader(node);
        auto robot_model = robot_model_loader.getModel();
        moveit::core::RobotState robot_state(robot_model);

        std::map<std::string, double> joint_positions;
        bool joint_received = false;

        auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            [&joint_positions, &joint_received](const sensor_msgs::msg::JointState::SharedPtr msg){
                for(size_t i=0; i<msg->name.size(); ++i){
                    joint_positions[msg->name[i]] = msg->position[i];
                }
                joint_received = true;
            });

        rclcpp::Rate rate(10);
        for(int i=0; i<50; ++i)
        {
            if(joint_received){
                robot_state.setVariablePositions(joint_positions);
                const moveit::core::LinkModel* ee_link = robot_model->getLinkModel(move_group.getEndEffectorLink());
                current_pose = eigenToMsgPose(robot_state.getGlobalLinkTransform(ee_link));
                got_pose = true;
                break;
            }
            rate.sleep();
        }
    }

    if(got_pose)
    {
        RCLCPP_INFO(node->get_logger(), "Current Pose:");
        RCLCPP_INFO(node->get_logger(), "X: %.7f Y: %.7f Z: %.7f",
                    current_pose.position.x,
                    current_pose.position.y,
                    current_pose.position.z);
        RCLCPP_INFO(node->get_logger(), "Orientation (x,y,z,w): %.7f, %.7f, %.7f, %.7f",
                    current_pose.orientation.x,
                    current_pose.orientation.y,
                    current_pose.orientation.z,
                    current_pose.orientation.w);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get valid robot pose. Make sure joint states are being published.");
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}