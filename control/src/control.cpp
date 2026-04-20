#include <memory>
#include <thread>
#include <vector>
#include <chrono>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "quoridor_interfaces/action/bot_move.hpp"

double deg2rad(double deg) { return deg * M_PI / 180.0; }

std::vector<double> PERCEPTION_WAYPOINT = {
    deg2rad(-72.94), deg2rad(-109.89), deg2rad(-9.94),
    deg2rad(-150.05), deg2rad(90.0), deg2rad(132.49)
};

std::vector<double> MOVEMENT_WAYPOINT = PERCEPTION_WAYPOINT;

constexpr double HOVER_OFFSET_Z = 0.03;

geometry_msgs::msg::Quaternion makeQuat(double x, double y, double z, double w)
{
    geometry_msgs::msg::Quaternion q;
    q.x = x; q.y = y; q.z = z; q.w = w;
    return q;
}

const geometry_msgs::msg::Quaternion ORI_PAWN =
    makeQuat(0.711, 0.703, -0.004, -0.012);

const geometry_msgs::msg::Quaternion ORI_WALL =
    makeQuat(0.707, 0.707, 0.0, 0.0);

geometry_msgs::msg::Pose hoverPose(const geometry_msgs::msg::Pose &pose)
{
    geometry_msgs::msg::Pose h = pose;
    h.position.z += HOVER_OFFSET_Z;
    return h;
}

geometry_msgs::msg::Pose withOrientation(
    geometry_msgs::msg::Pose pose,
    const geometry_msgs::msg::Quaternion &ori)
{
    pose.orientation = ori;
    return pose;
}

bool moveCartesian(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const geometry_msgs::msg::Pose &target,
    const std::string &name,
    rclcpp::Logger logger)
{
    std::vector<geometry_msgs::msg::Pose> waypoints = { target };

    moveit_msgs::msg::RobotTrajectory trajectory;

    double fraction = move_group.computeCartesianPath(
        waypoints, 0.005, 0.0, trajectory);

    if (fraction < 0.9)
    {
        RCLCPP_ERROR(logger, "Cartesian failed: %s (%.1f%%)", name.c_str(), fraction * 100);
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    return move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool moveToJoints(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const std::vector<double> &joints,
    const std::string &name,
    rclcpp::Logger logger)
{
    move_group.setJointValueTarget(joints);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(logger, "Planning failed: %s", name.c_str());
        return false;
    }

    return move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool moveToPose(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const geometry_msgs::msg::Pose &pose,
    const std::string &name,
    rclcpp::Logger logger)
{
    move_group.setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(logger, "Planning failed: %s", name.c_str());
        return false;
    }

    return move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("control");
    auto logger = node->get_logger();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "ur_onrobot_manipulator");

    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);

    rclcpp::sleep_for(std::chrono::seconds(2));

    auto gripper_pub = node->create_publisher<std_msgs::msg::String>(
        "/gripper/command", 10);

    bool gripper_done = false;

    auto gripper_sub = node->create_subscription<std_msgs::msg::String>(
        "/gripper/status", 10,
        [&](std_msgs::msg::String::SharedPtr msg)
        {
            if (msg->data == "done" || msg->data == "success")
                gripper_done = true;
        });

    auto sendGripper = [&](const std::string &cmd)
    {
        gripper_done = false;
        std_msgs::msg::String m;
        m.data = cmd;
        gripper_pub->publish(m);

        rclcpp::Rate r(20);
        while (!gripper_done && rclcpp::ok()) r.sleep();
    };

    bool first = true;

    using BotMove = quoridor_interfaces::action::BotMove;
    using GoalHandle = rclcpp_action::ServerGoalHandle<BotMove>;

    auto execute_cb = [&](std::shared_ptr<GoalHandle> gh)
    {
        const auto &goal = gh->get_goal();

        auto result = std::make_shared<BotMove::Result>();

        auto abort = [&](const std::string &msg)
        {
            result->result = false;
            gh->abort(result);
            RCLCPP_ERROR(logger, "%s", msg.c_str());
        };

        bool is_wall = goal->wall;

        const auto &ORI = is_wall ? ORI_WALL : ORI_PAWN;

        geometry_msgs::msg::Pose start = withOrientation(goal->start, ORI);
        geometry_msgs::msg::Pose end   = withOrientation(goal->end, ORI);

        geometry_msgs::msg::Pose start_hover = hoverPose(start);
        geometry_msgs::msg::Pose end_hover   = hoverPose(end);

        if (first)
        {
            if (!moveToJoints(move_group, PERCEPTION_WAYPOINT,
                "perception", logger))
                return abort("perception failed");

            sendGripper("open");
            first = false;
        }

        if (!moveToPose(move_group, start_hover,
            "start hover", logger))
            return abort("start hover failed");

        if (!moveCartesian(move_group, start,
            "descend start", logger))
            return abort("cartesian descend failed");

        sendGripper(is_wall ? "pickup_wall" : "pickup_pawn");

        if (!moveCartesian(move_group, start_hover,
            "lift start", logger))
            return abort("lift failed");

        if (!moveToJoints(move_group, MOVEMENT_WAYPOINT,
            "transit", logger))
            return abort("transit failed");

        if (!moveToPose(move_group, end_hover,
            "end hover", logger))
            return abort("end hover failed");

        if (!moveCartesian(move_group, end,
            "descend end", logger))
            return abort("descend end failed");

        sendGripper(is_wall ? "drop_wall" : "drop_pawn");

        if (!moveCartesian(move_group, end_hover,
            "lift end", logger))
            return abort("lift end failed");

        if (!moveToJoints(move_group, PERCEPTION_WAYPOINT,
            "return", logger))
            return abort("return failed");

        result->result = true;
        gh->succeed(result);

        RCLCPP_INFO(logger, "Completed");
    };

    auto server = rclcpp_action::create_server<BotMove>(
        node,
        "/quoridor/bot_execute",
        [](auto, auto){ return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
        [](auto){ return rclcpp_action::CancelResponse::ACCEPT; },
        [&](std::shared_ptr<GoalHandle> gh)
        {
            std::thread([&, gh]() { execute_cb(gh); }).detach();
        });

    RCLCPP_INFO(logger, "Control running");
    executor.spin();
    rclcpp::shutdown();
    return 0;
}