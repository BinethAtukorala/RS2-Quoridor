#include <memory>
#include <thread>
#include <vector>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
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

bool moveToWaypoint(moveit::planning_interface::MoveGroupInterface &move_group,
                    const std::vector<double> &target,
                    const std::string &name,
                    rclcpp::Logger logger)
{
    move_group.setStartStateToCurrentState();
    move_group.setJointValueTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group.plan(plan);

    bool success = (result == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(logger, "Executing %s", name.c_str());
        move_group.execute(plan);
    } else {
        RCLCPP_ERROR(logger, "Planning failed for %s", name.c_str());
    }

    rclcpp::sleep_for(std::chrono::seconds(2));
    return success;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "control",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto logger = rclcpp::get_logger("control");

    // Executor for MoveIt updates
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "ur_manipulator");

    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit_msgs::msg::CollisionObject ground;
    ground.id = "ground";
    ground.header.frame_id = "world";

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {2.0, 2.0, 0.1};

    geometry_msgs::msg::Pose ground_pose;
    ground_pose.orientation.w = 1.0;
    ground_pose.position.z = -0.05;

    ground.primitives.push_back(box);
    ground.primitive_poses.push_back(ground_pose);
    ground.operation = ground.ADD;

    planning_scene_interface.applyCollisionObject(ground);
    RCLCPP_INFO(logger, "Added ground collision object");

    std::vector<double> wp1 = toRadians({-70, -110, 20, -140, 90, 0});
    std::vector<double> wp2 = toRadians({-80, -120, 10, -130, 90, 10});
    std::vector<double> wp3 = toRadians({-60, -100, 30, -150, 90, -10});

    moveToWaypoint(move_group, wp1, "Waypoint 1", logger);
    moveToWaypoint(move_group, wp2, "Waypoint 2", logger);
    moveToWaypoint(move_group, wp3, "Waypoint 3", logger);

    rclcpp::shutdown();
    spinner.join();

    return 0;
}
