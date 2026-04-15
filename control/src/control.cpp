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
    deg2rad(-72.94),
    deg2rad(-109.89),
    deg2rad(-9.94),
    deg2rad(-150.05),
    deg2rad(90.0),
    deg2rad(132.49)
};

std::vector<double> MOVEMENT_WAYPOINT = {
    deg2rad(-72.94),
    deg2rad(-109.89),
    deg2rad(-9.94),
    deg2rad(-150.05),
    deg2rad(90.0),
    deg2rad(132.49)
};

constexpr double HOVER_OFFSET_Z = 0.02;

bool moveToJoints(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const std::vector<double> &joints,
    const std::string &name,
    rclcpp::Logger logger)
{
    move_group.setJointValueTarget(joints);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(logger, "Executing joints: %s", name.c_str());
        auto result = move_group.execute(plan);
        if (result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(logger, "Execution failed: %s", name.c_str());
            return false;
        }
    } else {
        RCLCPP_ERROR(logger, "Planning failed: %s", name.c_str());
        return false;
    }
    return true;
}

bool moveToPose(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const geometry_msgs::msg::Pose &pose,
    const std::string &name,
    rclcpp::Logger logger)
{
    move_group.setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto error_code = move_group.plan(plan);
    bool success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(logger, "Executing pose: %s", name.c_str());
        auto result = move_group.execute(plan);
        if (result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(logger, "Execution failed: %s", name.c_str());
            return false;
        }
    } else {
        RCLCPP_ERROR(logger, "Planning failed: %s (code: %d)",
                     name.c_str(), error_code.val);
        return false;
    }
    return true;
}

geometry_msgs::msg::Pose hoverPose(const geometry_msgs::msg::Pose &pose)
{
    geometry_msgs::msg::Pose h = pose;
    h.position.z += HOVER_OFFSET_Z;
    return h;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "control",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

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
        [&](std_msgs::msg::String::SharedPtr msg) {
            if (msg->data == "done"    || msg->data == "success" ||
                msg->data == "fail"    || msg->data == "wrong")
                gripper_done = true;
        }
    );

    auto publishGripperCommand = [&](const std::string &cmd) {
        gripper_done = false;
        std_msgs::msg::String msg;
        msg.data = cmd;
        gripper_pub->publish(msg);
        RCLCPP_INFO(logger, "Gripper: %s", cmd.c_str());
        rclcpp::Rate rate(20);
        while (!gripper_done && rclcpp::ok())
            rate.sleep();
    };

    bool first_move = true;

    using BotMove    = quoridor_interfaces::action::BotMove;
    using GoalHandle = rclcpp_action::ServerGoalHandle<BotMove>;

    auto send_feedback = [&](
        std::shared_ptr<GoalHandle> gh,
        float progress,
        const std::string &step)
    {
        auto fb = std::make_shared<BotMove::Feedback>();
        fb->progress = progress;
        gh->publish_feedback(fb);
        RCLCPP_INFO(logger, "[%.0f%%] %s", progress * 100.0f, step.c_str());
    };

    auto execute_cb = [&](std::shared_ptr<GoalHandle> gh)
    {
        const auto &goal = gh->get_goal();
        auto result = std::make_shared<BotMove::Result>();

        auto abort = [&](const std::string &reason) {
            result->result = false;
            gh->abort(result);
            RCLCPP_ERROR(logger, "BotMove ABORTED: %s", reason.c_str());
        };

        const bool is_wall = goal->wall;
        RCLCPP_INFO(logger, "BotMove received — %s move", is_wall ? "WALL" : "PAWN");

        geometry_msgs::msg::Pose start_hover = hoverPose(goal->start);
        geometry_msgs::msg::Pose end_hover   = hoverPose(goal->end);

        if (first_move) {
            send_feedback(gh, 0.05f, "Moving to perception waypoint (first move)");
            if (!moveToJoints(move_group, PERCEPTION_WAYPOINT, "perception_waypoint", logger))
                return abort("Failed at perception waypoint");

            publishGripperCommand("open");
            first_move = false;
        }

        send_feedback(gh, 0.18f, "Moving above start pose");
        if (!moveToPose(move_group, start_hover, "start_hover", logger))
            return abort("Failed at start hover");

        send_feedback(gh, 0.27f, "Descending to start pose");
        if (!moveToPose(move_group, goal->start, "start_pose", logger))
            return abort("Failed at start pose");

        send_feedback(gh, 0.36f, is_wall ? "Gripping wall" : "Gripping pawn");
        publishGripperCommand(is_wall ? "pickup_wall" : "pickup_pawn");

        send_feedback(gh, 0.45f, "Lifting to start hover");
        if (!moveToPose(move_group, start_hover, "start_hover_lift", logger))
            return abort("Failed lifting to start hover");

        send_feedback(gh, 0.54f, "Moving to transit waypoint");
        if (!moveToJoints(move_group, MOVEMENT_WAYPOINT, "movement_waypoint", logger))
            return abort("Failed at transit waypoint");

        send_feedback(gh, 0.63f, "Moving above end pose");
        if (!moveToPose(move_group, end_hover, "end_hover", logger))
            return abort("Failed at end hover");

        send_feedback(gh, 0.72f, "Descending to end pose");
        if (!moveToPose(move_group, goal->end, "end_pose", logger))
            return abort("Failed at end pose");

        send_feedback(gh, 0.81f, is_wall ? "Dropping wall" : "Dropping pawn");
        publishGripperCommand(is_wall ? "drop_wall" : "drop_pawn");

        send_feedback(gh, 0.90f, "Lifting to end hover");
        if (!moveToPose(move_group, end_hover, "end_hover_lift", logger))
            return abort("Failed lifting to end hover");

        send_feedback(gh, 1.00f, "Returning to perception waypoint");
        if (!moveToJoints(move_group, PERCEPTION_WAYPOINT, "perception_waypoint_return", logger))
            return abort("Failed returning to perception waypoint");

        result->result = true;
        gh->succeed(result);
        RCLCPP_INFO(logger, "BotMove completed successfully");
    };

    auto action_server = rclcpp_action::create_server<BotMove>(
        node,
        "/quoridor/bot_execute",

        [](const rclcpp_action::GoalUUID &,
           std::shared_ptr<const BotMove::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },

        [](std::shared_ptr<GoalHandle>) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },

        [&execute_cb](std::shared_ptr<GoalHandle> gh) {
            std::thread([&execute_cb, gh]() {
                execute_cb(gh);
            }).detach();
        }
    );

    RCLCPP_INFO(logger,
        "Control node ready — action server on /quoridor/bot_execute");

    executor.spin();
    rclcpp::shutdown();
    return 0;
}