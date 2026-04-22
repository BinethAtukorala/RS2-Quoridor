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

// ------------------------------------------------------------------ //
//  WALL PICKUP JOINT ANGLES
//
//  There are exactly 4 walls per game (shared between 'h' and 'v' types).
//  Each time a wall move arrives (regardless of h or v), the next entry
//  in this array is used as the robot's start pose for that move.
//
//  wall_pickup_index is incremented after every wall move, so:
//    wall move 1 → WALL_PICKUP_JOINTS[0]
//    wall move 2 → WALL_PICKUP_JOINTS[1]
//    wall move 3 → WALL_PICKUP_JOINTS[2]
//    wall move 4 → WALL_PICKUP_JOINTS[3]
//
//  TODO: drive the robot to each physical wall-stack position, read the
//        joint angles (e.g. via `ros2 topic echo /joint_states`), and
//        paste them below.  All six joints, in order.
// ------------------------------------------------------------------ //
const std::vector<std::vector<double>> WALL_PICKUP_JOINTS = {

    { -1.94215,-1.46597,-1.60545,-1.65821,1.58544,1.18939},
    { -1.91289,-1.68069,-1.42006,-1.67507,1.6095,1.19915},
    { -1.85773,-1.79817,-1.29396,-1.65578,1.59159,1.25853},
    { -1.86493,-1.97158,-1.07131,-1.69696,1.63539,1.21266},

};

// How far to retreat along tool axis for hover (metres)
constexpr double HOVER_OFFSET_M = 0.05;

// Ground plane height in metres (robot base frame Z)
constexpr double TABLE_Z_HEIGHT  = 0.0;   // TODO: set to your table surface height
constexpr double TABLE_THICKNESS = 0.05;

// Joint constraints — restricts motion to above the board only
struct JointBound { const char* name; double centre_deg; double tol_deg; };
constexpr JointBound JOINT_BOUNDS[] = {
    { "shoulder_pan_joint",  -60.3,  65.9 },
    { "shoulder_lift_joint", -115.2, 43.1 },
    { "elbow_joint",         -38.3,  60.4 },
};

// constexpr JointBound JOINT_BOUNDS[] = {
//     { "shoulder_pan_joint",  -60.3,  90.0 },
//     { "shoulder_lift_joint", -115.2, 75.0 },
//     { "elbow_joint",         -38.3,  90.0 },
// };

// ------------------------------------------------------------------ //
//  FIXED END-EFFECTOR ORIENTATIONS
// ------------------------------------------------------------------ //
geometry_msgs::msg::Quaternion makeQuat(double x, double y, double z, double w) {
    geometry_msgs::msg::Quaternion q;
    q.x = x; q.y = y; q.z = z; q.w = w;
    return q;
}

// Pawn and horizontal wall share the same gripper orientation
const geometry_msgs::msg::Quaternion ORI_PAWN_HWALL =
    makeQuat(1,0.-0.0000032,-0.0000076,0.0000159);

// Vertical wall — gripper rotated 90° around tool Z relative to above
const geometry_msgs::msg::Quaternion ORI_VWALL =
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
//  MOVE CARTESIAN  (straight line — single target)
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
        0.005,
        0.0,
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
//  MOVE CARTESIAN SEQUENCE  (smooth chained Cartesian path)
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

    double fraction = move_group.computeCartesianPath(
        poses,
        0.005,
        0.0,
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
//  HOVER POSE — retreats along tool -Z axis by `offset` metres.
// ------------------------------------------------------------------ //
geometry_msgs::msg::Pose hoverPose(
    const geometry_msgs::msg::Pose &pose,
    double offset = HOVER_OFFSET_M)
{
    tf2::Quaternion q(
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w);

    // offset > 0 → away from board (hover up); offset < 0 → toward board (descend)
    tf2::Vector3 world_offset = tf2::quatRotate(q, tf2::Vector3(0.0, 0.0, -offset));

    geometry_msgs::msg::Pose h = pose;
    h.position.x += world_offset.x();
    h.position.y += world_offset.y();
    h.position.z += world_offset.z();
    return h;
}

// ------------------------------------------------------------------ //
//  APPLY FIXED ORIENTATION
// ------------------------------------------------------------------ //
geometry_msgs::msg::Pose withOrientation(
    geometry_msgs::msg::Pose pose,
    const geometry_msgs::msg::Quaternion &ori)
{
    pose.orientation = ori;
    return pose;
}

// ------------------------------------------------------------------ //
//  FK HELPER — forward-kinematics a joint config to a Cartesian pose.
//  Used to compute the Cartesian pose that corresponds to a wall pickup
//  joint config, so we can chain it into a moveCartesianSequence call.
// ------------------------------------------------------------------ //
geometry_msgs::msg::Pose fkPose(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const std::vector<double> &joint_values)
{
    moveit::core::RobotStatePtr state = move_group.getCurrentState();
    state->setVariablePositions(move_group.getJointNames(), joint_values);
    const Eigen::Isometry3d &eef =
        state->getGlobalLinkTransform(move_group.getEndEffectorLink());

    geometry_msgs::msg::Pose pose;
    pose.position.x = eef.translation().x();
    pose.position.y = eef.translation().y();
    pose.position.z = eef.translation().z();
    Eigen::Quaterniond eq(eef.rotation());
    pose.orientation.x = eq.x();
    pose.orientation.y = eq.y();
    pose.orientation.z = eq.z();
    pose.orientation.w = eq.w();
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
        RCLCPP_INFO(logger, "Workspace joint constraints set");
    }

    // ================================================================ //
    //  GRIPPER
    // ================================================================ //
    auto gripper_pub = node->create_publisher<std_msgs::msg::String>(
        "/gripper/command",
        rclcpp::QoS(10).transient_local()
    );
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

    // Shared wall counter — incremented for every 'h' or 'v' move.
    // Indexes into WALL_PICKUP_JOINTS[0..3].
    // Game logic guarantees at most 4 wall moves per game.
    int wall_pickup_index = 0;

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
        //  Determine piece type
        //  'p' = pawn   → use goal start + goal end
        //  'h' = horizontal wall  \  use WALL_PICKUP_JOINTS[wall_pickup_index]
        //  'v' = vertical wall    /  for start; use goal end
        // ---------------------------------------------------------- //
        const std::string &pt = goal->piece_type;
        if (pt != "p" && pt != "h" && pt != "v") {
            return abort("Invalid piece_type '" + pt + "' — must be 'p', 'h', or 'v'");
        }

        const bool is_pawn  = (pt == "p");
        const bool is_hwall = (pt == "h");
        const bool is_vwall = (pt == "v");
        const bool is_wall  = is_hwall || is_vwall;

        // Pick fixed orientation based on piece type.
        // For vertical walls: pickup uses ORI_PAWN_HWALL (horizontal) so the
        // gripper matches the wall lying flat on the stack, then the end pose
        // uses ORI_VWALL so it is placed upright/vertical on the board.
        const auto &ori_pickup = ORI_PAWN_HWALL;                      // always pick up horizontal
        const auto &ori_place  = is_vwall ? ORI_VWALL : ORI_PAWN_HWALL;
        // Alias used by the shared pawn/wall pickup path below
        const auto &ori = ori_pickup;

        // ---------------------------------------------------------- //
        //  Validate wall counter before proceeding
        // ---------------------------------------------------------- //
        if (is_wall) {
            if (wall_pickup_index >= static_cast<int>(WALL_PICKUP_JOINTS.size())) {
                return abort("Wall pickup index out of bounds — more than 4 wall moves received");
            }
            RCLCPP_INFO(logger, "BotMove — piece: %s (wall slot %d/4)",
                        is_hwall ? "HORIZONTAL WALL" : "VERTICAL WALL",
                        wall_pickup_index + 1);
        } else {
            RCLCPP_INFO(logger, "BotMove — piece: PAWN");
        }

        // ---------------------------------------------------------- //
        //  Resolve start pose
        //
        //  PAWN  → use goal->start directly (lock orientation)
        //  WALL  → move to WALL_PICKUP_JOINTS[wall_pickup_index] via joint
        //          move, then compute the FK pose of that config to use as
        //          the Cartesian "current position" for the pickup sequence.
        //          The goal->start field is ignored for wall moves.
        // ---------------------------------------------------------- //
        geometry_msgs::msg::Pose start_fixed;
        geometry_msgs::msg::Pose start_hover;

        if (is_pawn) {
            start_fixed = withOrientation(goal->start, ori);
            start_hover = hoverPose(start_fixed);
        }
        // For walls, start_fixed / start_hover are computed after the joint
        // move in STEP 2W below (we need the robot at the joint config first).

        // Lock end pose orientation — vertical walls place with ORI_VWALL,
        // all other pieces (pawn, hwall) place with ORI_PAWN_HWALL.
        geometry_msgs::msg::Pose end_fixed = withOrientation(goal->end, ori_place);
        geometry_msgs::msg::Pose end_hover = hoverPose(end_fixed);

        // Gripper commands based on piece type
        const std::string cmd_pickup = is_pawn  ? "pickup_pawn" : "pickup_wall";
        const std::string cmd_drop   = is_pawn  ? "drop_pawn"   : "drop_wall";

        // ---------------------------------------------------------- //
        //  STEP 1 — Perception waypoint (first move only)
        // ---------------------------------------------------------- //
        if (first_move) {
            send_feedback(gh, 0.05f, "Initial perception waypoint");
            if (!moveToJoints(move_group, PERCEPTION_WAYPOINT, "perception_init", logger))
                return abort("Failed at initial perception waypoint");
            publishGripperCommand("open");
            first_move = false;
        }

        // ---------------------------------------------------------- //
        //  STEP 2P (pawn) — Free-space / Cartesian move to start hover
        // ---------------------------------------------------------- //
        if (is_pawn) {
            send_feedback(gh, 0.18f, "Moving to start hover (pawn)");
            if (!moveCartesianSequence(move_group,
                    { move_group.getCurrentPose().pose, start_hover },
                    "approach_start_hover", logger))
                return abort("Failed approaching start hover");
        }

        // ---------------------------------------------------------- //
        //  STEP 2W (wall) — Joint move to wall pickup slot, then compute
        //  the FK Cartesian pose so we can treat it exactly like a pawn
        //  start hover from here onward.
        //
        //  We go joint-space to the pickup slot because:
        //    a) The slot may be far from the current position — Cartesian
        //       would likely fail across that range.
        //    b) We don't need a straight-line path to get to the slot;
        //       we just need to arrive at a defined, repeatable config.
        //
        //  After the joint move we derive the Cartesian hover + contact
        //  poses from FK so the rest of the pickup sequence is identical
        //  to the pawn path.
        // ---------------------------------------------------------- //
        if (is_wall) {
            const auto &pickup_joints = WALL_PICKUP_JOINTS[wall_pickup_index];
            send_feedback(gh, 0.18f,
                std::string("Moving to wall pickup slot ") +
                std::to_string(wall_pickup_index + 1));

            // Joint angles are recorded at HOVER height (gripper 5 cm above
            // the wall). The joint move lands the robot directly at hover —
            // no free-space approach needed.
            //
            // FK of the joint config → start_hover in Cartesian space.
            // Negative hoverPose offset → start_fixed 5 cm below (contact).
            //
            // Flow:  joint_move → [AT hover] → descend → contact → pick → ascend
            if (!moveToJoints(move_group, pickup_joints, "wall_pickup_hover", logger))
                return abort("Failed moving to wall pickup hover config");

            start_hover = withOrientation(fkPose(move_group, pickup_joints), ori);
            start_fixed = hoverPose(start_hover, -HOVER_OFFSET_M);
        }

        // ---------------------------------------------------------- //
        //  STEP 3 — Cartesian descent: hover → contact (start)
        //
        //  At this point both pawn and wall have the robot at start_hover,
        //  so the descent is identical for both piece types.
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
        //  STEP 5 — Cartesian ascent: contact → hover (start)
        // ---------------------------------------------------------- //
        send_feedback(gh, 0.45f, "Cartesian contact → hover (start)");
        if (!moveCartesianSequence(move_group,
                { start_fixed, start_hover },
                "start_to_hover", logger))
            return abort("Failed Cartesian contact→hover at start");

        // ---------------------------------------------------------- //
        //  STEP 6 — Cartesian move to transit waypoint
        // ---------------------------------------------------------- //
        // send_feedback(gh, 0.54f, "Moving to transit waypoint");
        // {
        //     geometry_msgs::msg::Pose transit_pose = fkPose(move_group, MOVEMENT_WAYPOINT);
        //     move_group.setJointValueTarget(MOVEMENT_WAYPOINT);
        //     if (!moveCartesianSequence(move_group,
        //             { move_group.getCurrentPose().pose, transit_pose },
        //             "transit_waypoint", logger))
        //         return abort("Failed at transit waypoint");
        // }

        send_feedback(gh, 0.54f, "Moving to transit waypoint");
        {
            geometry_msgs::msg::Pose transit_pose = fkPose(move_group, MOVEMENT_WAYPOINT);
            transit_pose.orientation = ori_place;
            move_group.setJointValueTarget(MOVEMENT_WAYPOINT);
            if (!moveCartesianSequence(move_group,
                    { move_group.getCurrentPose().pose, transit_pose },
                    "transit_waypoint", logger))
                return abort("Failed at transit waypoint");
        }

        // ---------------------------------------------------------- //
        //  STEP 7 — Move to end hover
        // ---------------------------------------------------------- //
        send_feedback(gh, 0.63f, "Moving to end hover");
        if (!moveCartesianSequence(move_group,
                { move_group.getCurrentPose().pose, end_hover },
                "approach_end_hover", logger))
            return abort("Failed approaching end hover");

        // ---------------------------------------------------------- //
        //  STEP 8 — Cartesian descent: hover → contact (end)
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
        //  STEP 10 — Cartesian ascent: contact → hover (end)
        // ---------------------------------------------------------- //
        send_feedback(gh, 0.90f, "Cartesian contact → hover (end)");
        if (!moveCartesianSequence(move_group,
                { end_fixed, end_hover },
                "end_to_hover", logger))
            return abort("Failed Cartesian contact→hover at end");

        // ---------------------------------------------------------- //
        //  STEP 11 — Return to perception waypoint via transit
        // ---------------------------------------------------------- //
        // send_feedback(gh, 1.00f, "Returning to perception waypoint");
        // {
        //     geometry_msgs::msg::Pose return_pose = fkPose(move_group, MOVEMENT_WAYPOINT);
        //     move_group.setJointValueTarget(MOVEMENT_WAYPOINT);
        //     if (!moveCartesianSequence(move_group,
        //             { move_group.getCurrentPose().pose, return_pose },
        //             "perception_return", logger))
        //         return abort("Failed returning to perception waypoint");
        // }
        send_feedback(gh, 1.00f, "Returning to perception waypoint");
        {
            geometry_msgs::msg::Pose return_pose = fkPose(move_group, MOVEMENT_WAYPOINT);
            return_pose.orientation = ori_place;
            move_group.setJointValueTarget(MOVEMENT_WAYPOINT);
            if (!moveCartesianSequence(move_group,
                    { move_group.getCurrentPose().pose, return_pose },
                    "perception_return", logger))
                return abort("Failed returning to perception waypoint");
        }

        // ---------------------------------------------------------- //
        //  Advance wall counter AFTER a successful wall move.
        //  Only incremented on success so a retry uses the same slot.
        // ---------------------------------------------------------- //
        if (is_wall) {
            ++wall_pickup_index;
            RCLCPP_INFO(logger, "Wall pickup index advanced to %d", wall_pickup_index);
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