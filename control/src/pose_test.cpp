#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

void rotate90CCW(double &x, double &y)
{
    double tmp = x;
    x = -y;
    y = tmp;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pose_test_node");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    using moveit::planning_interface::MoveGroupInterface;
    using moveit::planning_interface::PlanningSceneInterface;

    MoveGroupInterface move_group(node, "ur_onrobot_manipulator");
    PlanningSceneInterface planning_scene_interface;

    RCLCPP_INFO(node->get_logger(), "Planning frame: %s",
                move_group.getPlanningFrame().c_str());

    rclcpp::sleep_for(std::chrono::seconds(2));

    struct WorkspaceBounds {
        double table_x = 1.0;
        double table_y = 1.0;
        double table_z = 0.1;
        double back_offset = 0.6;
        double side_offset = 0.6;
        double wall_height = 1.0;
    } workspace;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    std::vector<moveit_msgs::msg::ObjectColor> object_colors;
    std::string frame_id = move_group.getPlanningFrame();

    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = frame_id;
    table.id = "table";

    shape_msgs::msg::SolidPrimitive table_primitive;
    table_primitive.type = table_primitive.BOX;
    table_primitive.dimensions = {workspace.table_x, workspace.table_y, workspace.table_z};

    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 0.0;
    table_pose.position.y = 0.0;
    table_pose.position.z = -workspace.table_z / 2.0;
    table_pose.orientation.w = 1.0;

    table.primitives.push_back(table_primitive);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;
    collision_objects.push_back(table);

    moveit_msgs::msg::CollisionObject back_wall;
    back_wall.header.frame_id = frame_id;
    back_wall.id = "back_wall";

    shape_msgs::msg::SolidPrimitive back_primitive;
    back_primitive.type = back_primitive.BOX;
    back_primitive.dimensions = {0.1, workspace.table_y + 0.2, workspace.wall_height};

    double back_x = -workspace.back_offset;
    double back_y = 0.0;
    rotate90CCW(back_x, back_y);

    geometry_msgs::msg::Pose back_pose;
    back_pose.position.x = back_x;
    back_pose.position.y = back_y;
    back_pose.position.z = workspace.wall_height / 2.0;

    back_pose.orientation.w = cos(M_PI/4);
    back_pose.orientation.x = 0.0;
    back_pose.orientation.y = 0.0;
    back_pose.orientation.z = sin(M_PI/4);

    back_wall.primitives.push_back(back_primitive);
    back_wall.primitive_poses.push_back(back_pose);
    back_wall.operation = back_wall.ADD;
    collision_objects.push_back(back_wall);

    moveit_msgs::msg::ObjectColor back_color;
    back_color.id = "back_wall";
    back_color.color.r = 1.0;
    back_color.color.g = 0.0;
    back_color.color.b = 0.0;
    back_color.color.a = 0.8;
    object_colors.push_back(back_color);

    moveit_msgs::msg::CollisionObject left_wall;
    left_wall.header.frame_id = frame_id;
    left_wall.id = "left_wall";

    shape_msgs::msg::SolidPrimitive left_primitive;
    left_primitive.type = left_primitive.BOX;
    left_primitive.dimensions = {0.1, workspace.table_y + 0.2, workspace.wall_height};

    double left_x = -workspace.table_x / 2.0 + 0.5;
    double left_y = -workspace.side_offset;
    rotate90CCW(left_x, left_y);

    geometry_msgs::msg::Pose left_pose;
    left_pose.position.x = left_x;
    left_pose.position.y = left_y;
    left_pose.position.z = workspace.wall_height / 2.0;
    left_pose.orientation.w = 1.0;

    left_wall.primitives.push_back(left_primitive);
    left_wall.primitive_poses.push_back(left_pose);
    left_wall.operation = left_wall.ADD;
    collision_objects.push_back(left_wall);

    moveit_msgs::msg::ObjectColor left_color;
    left_color.id = "left_wall";
    left_color.color.r = 0.0;
    left_color.color.g = 1.0;
    left_color.color.b = 0.0;
    left_color.color.a = 0.8;
    object_colors.push_back(left_color);

    moveit_msgs::msg::CollisionObject right_wall;
    right_wall.header.frame_id = frame_id;
    right_wall.id = "right_wall";

    shape_msgs::msg::SolidPrimitive right_primitive;
    right_primitive.type = right_primitive.BOX;
    right_primitive.dimensions = {0.1, workspace.table_y + 0.2, workspace.wall_height};

    double right_x = workspace.table_x / 2.0 - 0.5;
    double right_y = workspace.side_offset;
    rotate90CCW(right_x, right_y);

    geometry_msgs::msg::Pose right_pose;
    right_pose.position.x = right_x;
    right_pose.position.y = right_y;
    right_pose.position.z = workspace.wall_height / 2.0;
    right_pose.orientation.w = 1.0;

    right_wall.primitives.push_back(right_primitive);
    right_wall.primitive_poses.push_back(right_pose);
    right_wall.operation = right_wall.ADD;
    collision_objects.push_back(right_wall);

    moveit_msgs::msg::ObjectColor right_color;
    right_color.id = "right_wall";
    right_color.color.r = 0.0;
    right_color.color.g = 0.0;
    right_color.color.b = 1.0;
    right_color.color.a = 0.8;
    object_colors.push_back(right_color);

    auto scene_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
    moveit_msgs::msg::PlanningScene planning_scene_msg;
    planning_scene_msg.is_diff = true;
    planning_scene_msg.object_colors = object_colors;
    scene_pub->publish(planning_scene_msg);

    RCLCPP_INFO(node->get_logger(), "Added safety environment with correct wall orientation");

    rclcpp::sleep_for(std::chrono::seconds(2));

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x =  0.43022458;
    target_pose.position.y = 0.09734114;
    target_pose.position.z = 0.23432327;
    target_pose.orientation.x = -0.50130461;
    target_pose.orientation.y = -0.49590339;
    target_pose.orientation.z = 0.49546997;
    target_pose.orientation.w = 0.50723074;

    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if(success) {
        RCLCPP_INFO(node->get_logger(), "Pose is reachable, executing move");
        move_group.move();
    } else {
        RCLCPP_WARN(node->get_logger(), "Pose is not reachable or might collide, skipping move");
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}