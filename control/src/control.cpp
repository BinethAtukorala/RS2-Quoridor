UPDATED CONTROL.CPP-
#include <memory>
#include <thread>
#include <vector>
#include <chrono>
#include <cmath>
#include <string>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "quoridor_interfaces/action/bot_move.hpp"

double deg2rad(double deg) { return deg * M_PI / 180.0; }

// ================================================================== //
//  TUNING — only change values in this section
// ================================================================== //

// Perception / transit waypoint (radians)
std::vector<double> PERCEPTION_WAYPOINT = {
    -1.32227, -1.66748, -0.218161, -2.85343, 1.54598, 0.24451
};

std::vector<double> MOVEMENT_WAYPOINT = {
    -1.1006,-1.3527,-1.39381,-1.94582,1.58668,0.482087
};

// How far to retreat along tool axis for hover (metres)
constexpr double HOVER_OFFSET_M = 0.05;

// Ground plane height in metres (robot base frame Z)
constexpr double TABLE_Z_HEIGHT  = 0.0;   // TODO: set to your table surface height
constexpr double TABLE_THICKNESS = 0.05;

// Joint constraints — restricts motion to above the board only
struct JointBound { const char* name; double centre_deg; double tol_deg; };
// constexpr JointBound JOINT_BOUNDS[] = {
//     { "shoulder_pan_joint",  -50.9,  51.3 },
//     { "shoulder_lift_joint", -95.0,  41.4 },
//     { "elbow_joint",         -54.5,  44.1 },
// };
constexpr JointBound JOINT_BOUNDS[] = {
    { "shoulder_pan_joint",  -60.3,  65.9 },
    { "shoulder_lift_joint", -115.2, 43.1 },
    { "elbow_joint",         -38.3,  60.4 },
};

// ------------------------------------------------------------------ //
//  FIXED END-EFFECTOR ORIENTATIONS
//  Locking orientation eliminates wrist spinning between moves.
//
//  PAWN / HORIZONTAL WALL — gripper faces one direction
//  TODO: move robot to a mid-board pose with gripper in the orientation
//        you want for pawn/horizontal-wall picks, run get_pose 2, paste here.
// ------------------------------------------------------------------ //
geometry_msgs::msg::Quaternion makeQuat(double x, double y, double z, double w) {
    geometry_msgs::msg::Quaternion q;
    q.x = x; q.y = y; q.z = z; q.w = w;
    return q;
}

// Pawn and horizontal wall share the same gripper orientation
const geometry_msgs::msg::Quaternion ORI_PAWN_HWALL =
    // makeQuat(0.727153, 0.6857, 0.0244286, 0.021608);  // TODO: replace with measured
    // makeQuat(0.711169,0.702905,-0.00450954,-0.0119684); worked this is movement waypoint
    makeQuat(1,0.-0.0000032,-0.0000076,0.0000159);

// Vertical wall — gripper rotated 90° around tool Z relative to above
const geometry_msgs::msg::Quaternion ORI_VWALL =
    // makeQuat(0.0, 0.0, 0.707, 0.707);  // TODO: replace with measured
    makeQuat(0.7071095,0.7071041,-0.0000050,-0.0000064);

// ================================================================== //

// ------------------------------------------------------------------ //
//  MOVE TO JOINT ANGLES
// ------------------------------------------------------------------ //
bool moveToJoints(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const std::vector<double> &joints,
    const std::string &name,
    rclcpp::Logger logger)
{
    move_group.setJointValueTarget(joints);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(logger, "Executing joints: %s", name.c_str());
        if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(logger, "Execution failed: %s", name.c_str());
            return false;
        }
    } else {
        RCLCPP_ERROR(logger, "Planning failed: %s", name.c_str());
        return false;
    }
    return true;
}

// ------------------------------------------------------------------ //
//  MOVE TO POSE  (free-space)
//  Used for longer moves where a straight-line path is not required,
//  e.g. returning from transit waypoint to a far hover pose.
// ------------------------------------------------------------------ //
bool moveToPose(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const geometry_msgs::msg::Pose &pose,
    const std::string &name,
    rclcpp::Logger logger)
{
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto ec = move_group.plan(plan);
    if (ec == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(logger, "Executing pose: %s", name.c_str());
        if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(logger, "Execution failed: %s", name.c_str());
            return false;
        }
    } else {
        RCLCPP_ERROR(logger, "Planning failed: %s (code: %d)", name.c_str(), ec.val);
        return false;
    }
    return true;
}

// ------------------------------------------------------------------ //
//  MOVE CARTESIAN  (straight line in world space — single target)
//  Used for single-step descend/ascend where the path is fully defined.
// ------------------------------------------------------------------ //
bool moveCartesian(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const geometry_msgs::msg::Pose &target,
    const std::string &name,
    rclcpp::Logger logger,
    double min_fraction = 0.95)
{
    std::vector<geometry_msgs::msg::Pose> waypoints = { target };
    moveit_msgs::msg::RobotTrajectory trajectory;

    double fraction = move_group.computeCartesianPath(
        waypoints,
        0.005,  // eef_step: 5 mm interpolation for smoother motion
        0.0,    // jump_threshold: disabled
        trajectory);

    RCLCPP_INFO(logger, "Cartesian %s: %.1f%% complete", name.c_str(), fraction * 100.0);

    if (fraction < min_fraction) {
        RCLCPP_ERROR(logger, "Cartesian path too short: %s (%.1f%% < %.0f%%)",
                     name.c_str(), fraction * 100.0, min_fraction * 100.0);
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Cartesian execution failed: %s", name.c_str());
        return false;
    }
    return true;
}

// ------------------------------------------------------------------ //
//  MOVE CARTESIAN SEQUENCE  (Option A — smooth chained Cartesian path)
//
//  Takes an ordered list of poses and computes a SINGLE Cartesian path
//  through all of them. This eliminates the IK discontinuity that occurs
//  when separate moveToPose + moveCartesian calls are chained, because:
//    - The planner sees the full path at once and cannot jump IK solutions
//      between waypoints.
//    - Velocity is continuous across intermediate waypoints (no stop/start).
//    - The eef traces a smooth straight-line sequence in world space.
//
//  Use this wherever you previously called moveToPose then moveCartesian
//  in sequence (hover → contact, or contact → hover).
// ------------------------------------------------------------------ //
bool moveCartesianSequence(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const std::vector<geometry_msgs::msg::Pose> &poses,
    const std::string &name,
    rclcpp::Logger logger,
    double min_fraction = 0.95)
{
    if (poses.empty()) {
        RCLCPP_ERROR(logger, "moveCartesianSequence: empty pose list for %s", name.c_str());
        return false;
    }

    moveit_msgs::msg::RobotTrajectory trajectory;

    // computeCartesianPath interpolates linearly between each consecutive
    // pair of poses with eef_step resolution. Using 5 mm gives ~10 points
    // per hover offset, producing visibly smooth motion.
    double fraction = move_group.computeCartesianPath(
        poses,
        0.005,  // eef_step: 5 mm interpolation
        0.0,    // jump_threshold: disabled (0 = allow any joint delta per step)
        trajectory);

    RCLCPP_INFO(logger, "Cartesian sequence %s: %.1f%% complete (%zu waypoints)",
                name.c_str(), fraction * 100.0, poses.size());

    if (fraction < min_fraction) {
        RCLCPP_ERROR(logger,
                     "Cartesian sequence too short: %s (%.1f%% < %.0f%%) — "
                     "check IK reachability for all poses in the sequence",
                     name.c_str(), fraction * 100.0, min_fraction * 100.0);
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Cartesian sequence execution failed: %s", name.c_str());
        return false;
    }
    return true;
}

// ------------------------------------------------------------------ //
//  HOVER POSE — retreats along tool -Z axis (correct for any tilt)
// ------------------------------------------------------------------ //
geometry_msgs::msg::Pose hoverPose(
    const geometry_msgs::msg::Pose &pose,
    double offset = HOVER_OFFSET_M)
{
    tf2::Quaternion q(
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w);

    tf2::Vector3 world_offset = tf2::quatRotate(q, tf2::Vector3(0.0, 0.0, -offset));

    geometry_msgs::msg::Pose h = pose;
    h.position.x += world_offset.x();
    h.position.y += world_offset.y();
    h.position.z += world_offset.z();
    return h;
}

// ------------------------------------------------------------------ //
//  APPLY FIXED ORIENTATION
//  Replaces the pose orientation with the fixed one for the piece type.
//  Position comes from game logic; orientation is always locked.
// ------------------------------------------------------------------ //
geometry_msgs::msg::Pose withOrientation(
    geometry_msgs::msg::Pose pose,
    const geometry_msgs::msg::Quaternion &ori)
{
    pose.orientation = ori;
    return pose;
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
    using moveit::planning_interface::PlanningSceneInterface;

    MoveGroupInterface move_group(node, "ur_onrobot_manipulator");
    PlanningSceneInterface planning_scene_interface;

    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(5);

    rclcpp::sleep_for(std::chrono::seconds(2));

    // ================================================================ //
    //  COLLISION OBJECT — ground plane
    // ================================================================ //
    {
        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = move_group.getPlanningFrame();
        table.id = "ground_plane";

        shape_msgs::msg::SolidPrimitive box;
        box.type = box.BOX;
        box.dimensions = { 3.0, 3.0, TABLE_THICKNESS };

        geometry_msgs::msg::Pose table_pose;
        table_pose.orientation.w = 1.0;
        table_pose.position.z    = TABLE_Z_HEIGHT - TABLE_THICKNESS / 2.0;

        table.primitives.push_back(box);
        table.primitive_poses.push_back(table_pose);
        table.operation = table.ADD;

        planning_scene_interface.applyCollisionObjects({table});
        RCLCPP_INFO(logger, "Ground plane added at z=%.3f", TABLE_Z_HEIGHT);
    }

    // ================================================================ //
    //  WORKSPACE JOINT CONSTRAINTS
    // ================================================================ //
    {
        moveit_msgs::msg::Constraints c;
        for (const auto &b : JOINT_BOUNDS) {
            moveit_msgs::msg::JointConstraint jc;
            jc.joint_name      = b.name;
            jc.position        = deg2rad(b.centre_deg);
            jc.tolerance_above = deg2rad(b.tol_deg);
            jc.tolerance_below = deg2rad(b.tol_deg);
            jc.weight          = 1.0;
            c.joint_constraints.push_back(jc);
        }
        move_group.setPathConstraints(c);
        // move_group.clearPathConstraints(c);
        RCLCPP_INFO(logger, "Workspace joint constraints set");
    }

    // ================================================================ //
    //  GRIPPER
    // ================================================================ //
    // auto gripper_pub = node->create_publisher<std_msgs::msg::String>("/gripper/command", 10);
    auto gripper_pub = node->create_publisher<std_msgs::msg::String>(
        "/gripper/command",
        rclcpp::QoS(10).transient_local()
    );
    // bool gripper_done = false;
    std::atomic<bool> gripper_done{false};

    auto gripper_sub = node->create_subscription<std_msgs::msg::String>(
        "/gripper/status", 10,
        [&](std_msgs::msg::String::SharedPtr msg) {
            if (msg->data == "done"  || msg->data == "success" ||
                msg->data == "fail"  || msg->data == "wrong")
                gripper_done = true;
        });

    auto publishGripperCommand = [&](const std::string &cmd) {
        gripper_done = false;
        std_msgs::msg::String m;
        m.data = cmd;
        gripper_pub->publish(m);
        RCLCPP_INFO(logger, "Gripper: %s", cmd.c_str());
        rclcpp::Rate rate(20);
        while (!gripper_done && rclcpp::ok())
            rate.sleep();
    };

    // ================================================================ //
    //  ACTION SERVER
    // ================================================================ //
    bool first_move = true;

    using BotMove    = quoridor_interfaces::action::BotMove;
    using GoalHandle = rclcpp_action::ServerGoalHandle<BotMove>;

    auto send_feedback = [&](std::shared_ptr<GoalHandle> gh,
                              float progress, const std::string &step) {
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

        // ---------------------------------------------------------- //
        //  Determine piece type from the single char
        //  'p' = pawn, 'h' = horizontal wall, 'v' = vertical wall
        // ---------------------------------------------------------- //
        const std::string &pt = goal->piece_type;
        if (pt != "p" && pt != "h" && pt != "v") {
            return abort("Invalid piece_type '" + pt + "' — must be 'p', 'h', or 'v'");
        }

        const bool is_pawn  = (pt == "p");
        const bool is_hwall = (pt == "h");
        const bool is_vwall = (pt == "v");

        // Pick fixed orientation based on piece type
        // Pawn and horizontal wall use the same orientation
        const auto &ori = is_vwall ? ORI_VWALL : ORI_PAWN_HWALL;

        RCLCPP_INFO(logger, "BotMove — piece: %s",
                    is_pawn ? "PAWN" : (is_hwall ? "HORIZONTAL WALL" : "VERTICAL WALL"));

        // Lock position from goal, replace orientation with fixed value
        geometry_msgs::msg::Pose start_fixed = withOrientation(goal->start, ori);
        geometry_msgs::msg::Pose end_fixed   = withOrientation(goal->end,   ori);

        // Hover poses computed from locked poses
        geometry_msgs::msg::Pose start_hover = hoverPose(start_fixed);
        geometry_msgs::msg::Pose end_hover   = hoverPose(end_fixed);

        // Gripper commands based on piece type
        const std::string cmd_pickup = is_pawn  ? "pickup_pawn" :
                                       is_hwall ? "pickup_wall" : "pickup_wall";
        const std::string cmd_drop   = is_pawn  ? "drop_pawn"   :
                                       is_hwall ? "drop_wall"   : "drop_wall";

        // ---------------------------------------------------------- //
        //  STEP 1 — Perception waypoint (first move only)
        // ---------------------------------------------------------- //
        if (first_move) {
            send_feedback(gh, 0.05f, "Initial perception waypoint");
            if (!moveToJoints(move_group, PERCEPTION_WAYPOINT, "perception_init", logger))
                return abort("Failed at initial perception waypoint");
            // geometry_msgs::msg::Pose perception_pose = move_group.getCurrentPose().pose;
            // perception_pose = withOrientation(perception_pose, ori);
            // if (!moveCartesianSequence(move_group,
            //         { move_group.getCurrentPose().pose, perception_pose },
            //         "perception_init", logger))
            //     return abort("Failed at initial perception waypoint");
            publishGripperCommand("open");
            first_move = false;
        }

        // ---------------------------------------------------------- //
        //  STEP 2 — Free-space move from transit waypoint to start hover.
        //
        //  We still use moveToPose here because the robot is arriving
        //  from a joint-space position (PERCEPTION_WAYPOINT). A free-space
        //  planner handles this longer, unconstrained arc better than
        //  Cartesian, which would fail if the straight-line path from the
        //  waypoint tip position to start_hover is not fully reachable.
        // ---------------------------------------------------------- //
        send_feedback(gh, 0.18f, "Moving to start hover (free-space)");
        // if (!moveToPose(move_group, start_hover, "approach_start_hover", logger))
        //     return abort("Failed approaching start hover");
        if (!moveCartesianSequence(move_group,
                { move_group.getCurrentPose().pose, start_hover },
                "approach_start_hover", logger))
            return abort("Failed approaching start hover");

        // ---------------------------------------------------------- //
        //  STEP 3 — Smooth Cartesian descent: hover → contact (Option A)
        //
        //  Single computeCartesianPath call through {start_hover, start_fixed}.
        //  The planner sees both points at once, so IK is solved continuously
        //  and the tool traces a straight line with no stop between hover
        //  and the board surface.
        // ---------------------------------------------------------- //
        send_feedback(gh, 0.27f, "Cartesian hover → contact (start)");
        if (!moveCartesianSequence(move_group,
                { start_hover, start_fixed },
                "hover_to_start", logger))
            return abort("Failed Cartesian hover→contact at start");

        // ---------------------------------------------------------- //
        //  STEP 4 — Close gripper
        // ---------------------------------------------------------- //
        send_feedback(gh, 0.36f, std::string("Picking ") + (is_pawn ? "pawn" : "wall"));
        publishGripperCommand(cmd_pickup);

        // ---------------------------------------------------------- //
        //  STEP 5 — Smooth Cartesian ascent: contact → hover (Option A)
        //
        //  Mirrors step 3 in reverse. Single path keeps velocity smooth
        //  through the ascent without a replanning pause at the hover point.
        // ---------------------------------------------------------- //
        send_feedback(gh, 0.45f, "Cartesian contact → hover (start)");
        if (!moveCartesianSequence(move_group,
                { start_fixed, start_hover },
                "start_to_hover", logger))
            return abort("Failed Cartesian contact→hover at start");

        // ---------------------------------------------------------- //
        //  STEP 6 — Joint move to transit waypoint
        //
        //  Joint-space move is correct here: we are travelling a large arc
        //  from one side of the board to the other (or through a home
        //  configuration). Cartesian would be unnecessarily restrictive and
        //  may not find a valid straight-line path across that distance.
        // ---------------------------------------------------------- //
        send_feedback(gh, 0.54f, "Moving to transit waypoint");
        // if (!moveToJoints(move_group, MOVEMENT_WAYPOINT, "transit_waypoint", logger))
        //     return abort("Failed at transit waypoint");
        {
            move_group.setJointValueTarget(MOVEMENT_WAYPOINT);
            geometry_msgs::msg::Pose transit_pose;
            moveit::core::RobotStatePtr state = move_group.getCurrentState();
            state->setVariablePositions(move_group.getJointNames(), MOVEMENT_WAYPOINT);
            const Eigen::Isometry3d &eef = state->getGlobalLinkTransform(move_group.getEndEffectorLink());
            transit_pose.position.x    = eef.translation().x();
            transit_pose.position.y    = eef.translation().y();
            transit_pose.position.z    = eef.translation().z();
            Eigen::Quaterniond eq(eef.rotation());
            transit_pose.orientation.x = eq.x();
            transit_pose.orientation.y = eq.y();
            transit_pose.orientation.z = eq.z();
            transit_pose.orientation.w = eq.w();
            if (!moveCartesianSequence(move_group,
                    { move_group.getCurrentPose().pose, transit_pose },
                    "transit_waypoint", logger))
                return abort("Failed at transit waypoint");
        }

        // ---------------------------------------------------------- //
        //  STEP 7 — Free-space move from transit waypoint to end hover.
        //  Same reasoning as step 2.
        // ---------------------------------------------------------- //
        send_feedback(gh, 0.63f, "Moving to end hover (free-space)");
        // if (!moveToPose(move_group, end_hover, "approach_end_hover", logger))
        //     return abort("Failed approaching end hover");
        if (!moveCartesianSequence(move_group,
                { move_group.getCurrentPose().pose, end_hover },
                "approach_end_hover", logger))
            return abort("Failed approaching end hover");

        // ---------------------------------------------------------- //
        //  STEP 8 — Smooth Cartesian descent: hover → contact (Option A)
        //
        //  Identical pattern to step 3, applied to the end (place) pose.
        //  Chains hover and contact into one smooth continuous path.
        // ---------------------------------------------------------- //
        send_feedback(gh, 0.72f, "Cartesian hover → contact (end)");
        if (!moveCartesianSequence(move_group,
                { end_hover, end_fixed },
                "hover_to_end", logger))
            return abort("Failed Cartesian hover→contact at end");

        // ---------------------------------------------------------- //
        //  STEP 9 — Open gripper
        // ---------------------------------------------------------- //
        send_feedback(gh, 0.81f, std::string("Placing ") + (is_pawn ? "pawn" : "wall"));
        publishGripperCommand(cmd_drop);

        // ---------------------------------------------------------- //
        //  STEP 10 — Smooth Cartesian ascent: contact → hover (Option A)
        //
        //  Mirrors step 8. Single-path ascent from placed piece back to
        //  hover height before returning to transit waypoint.
        // ---------------------------------------------------------- //
        send_feedback(gh, 0.90f, "Cartesian contact → hover (end)");
        if (!moveCartesianSequence(move_group,
                { end_fixed, end_hover },
                "end_to_hover", logger))
            return abort("Failed Cartesian contact→hover at end");

        // ---------------------------------------------------------- //
        //  STEP 11 — Return to perception waypoint
        // ---------------------------------------------------------- //
        send_feedback(gh, 1.00f, "Returning to perception waypoint");
        // if (!moveToJoints(move_group, MOVEMENT_WAYPOINT, "perception_return", logger))
        //     return abort("Failed returning to perception waypoint");
        {
            move_group.setJointValueTarget(MOVEMENT_WAYPOINT);
            geometry_msgs::msg::Pose return_pose;
            moveit::core::RobotStatePtr state = move_group.getCurrentState();
            state->setVariablePositions(move_group.getJointNames(), MOVEMENT_WAYPOINT);
            const Eigen::Isometry3d &eef = state->getGlobalLinkTransform(move_group.getEndEffectorLink());
            return_pose.position.x    = eef.translation().x();
            return_pose.position.y    = eef.translation().y();
            return_pose.position.z    = eef.translation().z();
            Eigen::Quaterniond eq(eef.rotation());
            return_pose.orientation.x = eq.x();
            return_pose.orientation.y = eq.y();
            return_pose.orientation.z = eq.z();
            return_pose.orientation.w = eq.w();
            if (!moveCartesianSequence(move_group,
                    { move_group.getCurrentPose().pose, return_pose },
                    "perception_return", logger))
                return abort("Failed returning to perception waypoint");
        }

        result->result = true;
        gh->succeed(result);
        RCLCPP_INFO(logger, "BotMove completed successfully");
    };

    auto action_server = rclcpp_action::create_server<BotMove>(
        node, "/quoridor/bot_execute",
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const BotMove::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<GoalHandle>) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [&execute_cb](std::shared_ptr<GoalHandle> gh) {
            std::thread([&execute_cb, gh]() { execute_cb(gh); }).detach();
        }
    );

    RCLCPP_INFO(logger, "Control node ready — action server on /quoridor/bot_execute");
    executor.spin();
    rclcpp::shutdown();
    return 0;
}