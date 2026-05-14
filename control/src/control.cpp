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
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
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
    // -1.32227, -1.66748, -0.218161, -2.85343, 1.54598, 0.24451
    // -1.48368,-1.47431,-0.72157,-2.45629,1.60519,0.0981183
    -1.47639,-1.42106,-0.761994,-2.46893,1.60461,0.105417
};

std::vector<double> MOVEMENT_WAYPOINT = {
    -1.1006,-1.3527,-1.39381,-1.94582,1.58668,0.482087
};

// ------------------------------------------------------------------ //
//  WALL PICKUP JOINT ANGLES
// ------------------------------------------------------------------ //

// const std::vector<std::vector<double>> WALL_PICKUP_JOINTS = {
//     {-1.93956,-1.56641,-1.504,-1.62645,1.62396,1.11445},
//     {-1.892,-1.71207,-1.34779,-1.63452,1.62301,1.16183},
//     {-1.8558,-1.86483,-1.15782,-1.66983,1.62212,1.19775},
//     {-1.82511,-2.04725,-0.89662,-1.74706,1.62125,1.22808},
// };

const std::vector<std::vector<double>> WALL_PICKUP_JOINTS = {
    {-1.92576,-1.60661,-1.43416,-1.69649,1.6365,1.21335},
    {-1.88431,-1.75111,-1.27272,-1.71067,1.6373,1.25467},
    {-1.84482,-1.91192,-1.06507,-1.7549,1.63792,1.29394},
    {-1.81929,-2.10952,-0.769817,-1.8508,1.63814,1.31901},
};

// How far to retreat along tool axis for hover (metres)
constexpr double HOVER_OFFSET_M = 0.1;

// Ground plane height in metres (robot base frame Z)
constexpr double TABLE_Z_HEIGHT  = 0.0;
constexpr double TABLE_THICKNESS = 0.05;

// How many times to retry a failed gripper pickup before aborting
constexpr int MAX_PICKUP_RETRIES = 3;

// Joint constraints — restricts motion to above the board only
struct JointBound { const char* name; double centre_deg; double tol_deg; };
constexpr JointBound JOINT_BOUNDS[] = {
    { "shoulder_pan_joint",  -60.3,  65.9 },
    { "shoulder_lift_joint", -115.2, 43.1 },
    { "elbow_joint",         -38.3,  60.4 },
};

// ------------------------------------------------------------------ //
//  FIXED END-EFFECTOR ORIENTATIONS
// ------------------------------------------------------------------ //
geometry_msgs::msg::Quaternion makeQuat(double x, double y, double z, double w) {
    geometry_msgs::msg::Quaternion q;
    q.x = x; q.y = y; q.z = z; q.w = w;
    return q;
}

const geometry_msgs::msg::Quaternion ORI_PAWN_HWALL =
    makeQuat(1,-0.0000032,-0.0000076,0.0000159);

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
    double min_fraction = 0.0)
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
//  FK HELPER
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
    move_group.setGoalJointTolerance(1e-4);

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

    // Stores the last status string received: "done", "success", "fail", "wrong"
    std::atomic<bool>  gripper_done{false};
    std::string        gripper_last_status;
    std::mutex         gripper_status_mutex;

    auto gripper_sub = node->create_subscription<std_msgs::msg::String>(
        "/gripper/status", 10,
        [&](std_msgs::msg::String::SharedPtr msg) {
            if (msg->data == "done"  || msg->data == "success" ||
                msg->data == "fail"  || msg->data == "wrong") {
                {
                    std::lock_guard<std::mutex> lock(gripper_status_mutex);
                    gripper_last_status = msg->data;
                }
                gripper_done = true;
            }
        });

    // Returns the status string so callers can distinguish success/fail/wrong.
    auto publishGripperCommand = [&](const std::string &cmd) -> std::string {
        gripper_done = false;
        {
            std::lock_guard<std::mutex> lock(gripper_status_mutex);
            gripper_last_status = "";
        }
        std_msgs::msg::String m;
        m.data = cmd;
        gripper_pub->publish(m);
        RCLCPP_INFO(logger, "Gripper: %s", cmd.c_str());
        rclcpp::Rate rate(20);
        while (!gripper_done && rclcpp::ok())
            rate.sleep();
        std::lock_guard<std::mutex> lock(gripper_status_mutex);
        return gripper_last_status;
    };

    // ================================================================ //
    //  BOARD DETECTION SUBSCRIBER
    //  /perception/board_state  → std_msgs/Int32MultiArray
    //  data[0] == 1 → board detected, data[0] == 0 → not detected
    // ================================================================ //
    std::atomic<int>  board_detected{-1};   // -1 = no message yet
    std::mutex        board_mutex;

    auto board_sub = node->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/perception/board_state", 10,
        [&](std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            if (!msg->data.empty()) {
                std::lock_guard<std::mutex> lock(board_mutex);
                board_detected = 1;
            }
        });

    // Helper: move to perception waypoint and wait until board is detected.
    // Tries up to `max_attempts` times (each attempt = joint move + wait).
    // Returns true if board is detected, false if all attempts fail.
    auto ensureBoardDetected = [&](int max_attempts = 3) -> bool {
        for (int attempt = 1; attempt <= max_attempts; ++attempt) {
            RCLCPP_INFO(logger, "Board detection attempt %d/%d — moving to perception waypoint",
                        attempt, max_attempts);

            if (!moveToJoints(move_group, PERCEPTION_WAYPOINT,
                              "perception_waypoint_detection", logger)) {
                RCLCPP_ERROR(logger, "Failed to move to perception waypoint (attempt %d)", attempt);
                continue;
            }

            // Reset and wait up to 3 seconds for a fresh board_state message.
            {
                std::lock_guard<std::mutex> lock(board_mutex);
                board_detected = -1;
            }

            rclcpp::Rate wait_rate(20);
            int ticks = 0;
            constexpr int MAX_TICKS = 60;   // 3 seconds at 20 Hz

            while (rclcpp::ok() && ticks < MAX_TICKS) {
                {
                    std::lock_guard<std::mutex> lock(board_mutex);
                    if (board_detected != -1) break;
                }
                wait_rate.sleep();
                ++ticks;
            }

            {
                std::lock_guard<std::mutex> lock(board_mutex);
                if (board_detected) {
                    RCLCPP_INFO(logger, "Board detected on attempt %d", attempt);
                    return true;
                }
                RCLCPP_WARN(logger, "Board NOT detected on attempt %d (value=%d)",
                            attempt, board_detected.load());
            }
        }
        RCLCPP_ERROR(logger, "Board detection failed after %d attempts", max_attempts);
        return false;
    };

    // ================================================================ //
    //  STARTUP — move to perception waypoint and verify board detection
    // ================================================================ //
    if (!ensureBoardDetected()) {
        RCLCPP_ERROR(logger, "WARNING: Board not detected at startup — continuing anyway");
    }

    // ================================================================ //
    //  WALL SLOT TEST SUBSCRIBER
    //  Publish a slot number (1–4) to drive the robot to that wall pickup
    //  joint config without needing a full BotMove action goal. Use this
    //  to verify each slot is reachable before a real game run.
    //
    //  Example:
    //    ros2 topic pub --once /quoridor/test_wall_pickup std_msgs/msg/Int32 "{data: 1}"
    // ================================================================ //
    std::atomic<bool> test_move_active{false};

    auto wall_test_sub = node->create_subscription<std_msgs::msg::Int32>(
        "/quoridor/test_wall_pickup", 10,
        [&](std_msgs::msg::Int32::SharedPtr msg) {
            int slot = msg->data;  // 1-indexed
            if (slot < 1 || slot > static_cast<int>(WALL_PICKUP_JOINTS.size())) {
                RCLCPP_ERROR(logger,
                    "test_wall_pickup: slot %d out of range (1–%zu)",
                    slot, WALL_PICKUP_JOINTS.size());
                return;
            }
            if (test_move_active.exchange(true)) {
                RCLCPP_WARN(logger, "test_wall_pickup: a test move is already running — ignoring");
                return;
            }
            // Run in a detached thread so the subscriber callback returns immediately.
            std::thread([&, slot]() {
                RCLCPP_INFO(logger, "test_wall_pickup: moving to slot %d", slot);
                const auto &pickup_joints = WALL_PICKUP_JOINTS[slot - 1];

                // Disable board-zone constraints for the same reason as Step 2W.
                move_group.clearPathConstraints();

                bool ok = moveToJoints(move_group, pickup_joints,
                    "test_wall_slot_" + std::to_string(slot), logger);

                // Re-apply workspace constraints after test move.
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
                }

                if (ok) {
                    RCLCPP_INFO(logger, "test_wall_pickup: slot %d reached — check position, then publish next slot or move on", slot);
                } else {
                    RCLCPP_ERROR(logger, "test_wall_pickup: slot %d FAILED — check joint bounds vs WALL_PICKUP_JOINTS values", slot);
                }
                test_move_active = false;
            }).detach();
        });
        
    // ================================================================ //
    //  ACTION SERVER
    // ================================================================ //
    bool first_move = true;
    int  wall_pickup_index = 0;

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

        const std::string &pt = goal->piece_type;
        if (pt != "p" && pt != "h" && pt != "v") {
            return abort("Invalid piece_type '" + pt + "' — must be 'p', 'h', or 'v'");
        }

        const bool is_pawn  = (pt == "p");
        const bool is_hwall = (pt == "h");
        const bool is_vwall = (pt == "v");
        const bool is_wall  = is_hwall || is_vwall;

        const auto &ori_pickup = ORI_PAWN_HWALL;
        const auto &ori_place  = is_vwall ? ORI_VWALL : ORI_PAWN_HWALL;
        const auto &ori        = ori_pickup;

    if (is_wall) {
        // wall_pickup_index cycles 0→1→2→3→0→... — no hard limit.
        wall_pickup_index = wall_pickup_index % static_cast<int>(WALL_PICKUP_JOINTS.size());
        RCLCPP_INFO(logger, "BotMove — piece: %s (wall slot %d/4)",
                    is_hwall ? "HORIZONTAL WALL" : "VERTICAL WALL",
                    wall_pickup_index + 1);
        } else {
            RCLCPP_INFO(logger, "BotMove — piece: PAWN");
        }

        geometry_msgs::msg::Pose start_fixed;
        geometry_msgs::msg::Pose start_hover;

        constexpr double PAWN_Z = 0.07;
        constexpr double WALL_Z = 0.06;
        

        if (is_pawn) {
            start_fixed = withOrientation(goal->start, ori);
            start_fixed.position.z = PAWN_Z;
            start_hover = hoverPose(start_fixed);
        }

        geometry_msgs::msg::Pose end_fixed = withOrientation(goal->end, ori_place);
        end_fixed.position.z = is_pawn ? PAWN_Z : WALL_Z;
        geometry_msgs::msg::Pose end_hover = hoverPose(end_fixed);

        const std::string cmd_pickup = is_pawn  ? "pickup_pawn" : "pickup_wall";
        const std::string cmd_drop   = is_pawn  ? "drop_pawn"   : "drop_wall";

        // ---------------------------------------------------------- //
        //  STEP 1 — Perception waypoint (first move only)
        //  Also verify board is detected before proceeding.
        // ---------------------------------------------------------- //
        if (first_move) {
            send_feedback(gh, 0.05f, "Initial perception waypoint + board detection");
            if (!ensureBoardDetected()) {
                RCLCPP_WARN(logger, "Board not confirmed at first move — proceeding anyway");
            }
            publishGripperCommand("open");
            first_move = false;
        }

        // ---------------------------------------------------------- //
        //  STEP 2P (pawn) — Move to start hover
        // ---------------------------------------------------------- //
        if (is_pawn) {
            send_feedback(gh, 0.18f, "Moving to start hover (pawn)");
            if (!moveCartesianSequence(move_group,
                    { move_group.getCurrentPose().pose, start_hover },
                    "approach_start_hover", logger))
                return abort("Failed approaching start hover");
        }

// ---------------------------------------------------------- //
        //  STEP 2W (wall) — Joint move to wall pickup slot
        //  Workspace joint constraints are cleared before this move
        //  because the wall rack is outside the board zone — the
        //  JOINT_BOUNDS would otherwise reject the pickup joint targets.
        //  Constraints are re-applied immediately after the joint move.
        // ---------------------------------------------------------- //
        if (is_wall) {
            const auto &pickup_joints = WALL_PICKUP_JOINTS[wall_pickup_index];
            send_feedback(gh, 0.18f,
                std::string("Moving to wall pickup slot ") +
                std::to_string(wall_pickup_index + 1));

            // Disable board-zone constraints — wall rack is outside that region.
            move_group.clearPathConstraints();

            if (!moveToJoints(move_group, pickup_joints, "wall_pickup_hover", logger)) {
                // Re-apply before aborting so subsequent moves are still safe.
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
                return abort("Failed moving to wall pickup hover config");
            }

            // Re-apply workspace constraints now that we're at the pickup pose.
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
            }

            start_hover = withOrientation(fkPose(move_group, pickup_joints), ori);
            start_fixed = hoverPose(start_hover, -0.06);
        }
        // ---------------------------------------------------------- //
        //  STEPS 3–5 — Pickup with retry loop
        //
        //  On a failed pickup (wrong object or nothing grasped):
        //    1. Ascend back to start_hover  (Cartesian)
        //    2. Open gripper
        //    3. Descend again to start_fixed
        //    4. Retry gripper close
        //
        //  For wall moves, the hover pose is already set above so retries
        //  simply re-descend from the same hover; no joint move needed.
        // ---------------------------------------------------------- //
        bool pickup_ok = false;

        for (int attempt = 1; attempt <= MAX_PICKUP_RETRIES; ++attempt) {
            if (attempt > 1) {
                RCLCPP_WARN(logger, "Pickup retry %d/%d", attempt, MAX_PICKUP_RETRIES);
            }

            // Step 3 — Descend: hover → contact
            send_feedback(gh, 0.27f,
                          std::string("Cartesian hover → contact (attempt ") +
                          std::to_string(attempt) + ")");
            if (!moveCartesianSequence(move_group,
                    { start_hover, start_fixed },
                    "hover_to_start", logger))
                return abort("Failed Cartesian hover→contact at start");

            // Step 4 — Close gripper
            send_feedback(gh, 0.36f,
                          std::string("Picking ") + (is_pawn ? "pawn" : "wall") +
                          " (attempt " + std::to_string(attempt) + ")");
            std::string pickup_status = publishGripperCommand(cmd_pickup);

            if (pickup_status == "success") {
                // Good grip — proceed
                pickup_ok = true;
                break;
            }

            // Bad grip — ascend, open, and retry (or abort if out of attempts)
            RCLCPP_WARN(logger, "Pickup attempt %d failed (status: %s) — ascending and reopening",
                        attempt, pickup_status.c_str());

            // Ascend back to hover
            if (!moveCartesianSequence(move_group,
                    { start_fixed, start_hover },
                    "retry_ascend", logger)) {
                return abort("Failed to ascend after pickup failure");
            }

            // Open gripper
            publishGripperCommand("open");

            if (attempt == MAX_PICKUP_RETRIES) {
                return abort("Pickup failed after " + std::to_string(MAX_PICKUP_RETRIES) + " attempts");
            }

            // Small pause before next attempt
            rclcpp::sleep_for(std::chrono::milliseconds(300));
        }

        // ---------------------------------------------------------- //
        //  STEP 5 — Ascend: contact → hover (start)
        //  (only reached on success — retries ascend inside the loop)
        // ---------------------------------------------------------- //
        send_feedback(gh, 0.45f, "Cartesian contact → hover (start)");
        if (!moveCartesianSequence(move_group,
                { start_fixed, start_hover },
                "start_to_hover", logger))
            return abort("Failed Cartesian contact→hover at start");

        // ---------------------------------------------------------- //
        //  STEP 6 — Transit waypoint
        // ---------------------------------------------------------- //
        // send_feedback(gh, 0.54f, "Moving to transit waypoint");
        // {
        //     geometry_msgs::msg::Pose transit_pose = fkPose(move_group, MOVEMENT_WAYPOINT);
        //     transit_pose.orientation = ori_place;
        //     move_group.setJointValueTarget(MOVEMENT_WAYPOINT);
        //     if (!moveCartesianSequence(move_group,
        //             { move_group.getCurrentPose().pose, transit_pose },
        //             "transit_waypoint", logger))
        //         return abort("Failed at transit waypoint");
        // }

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
        //  STEP 10 — Ascent: contact → hover (end)
        // ---------------------------------------------------------- //
        send_feedback(gh, 0.90f, "Cartesian contact → hover (end)");
        if (!moveCartesianSequence(move_group,
                { end_fixed, end_hover },
                "end_to_hover", logger))
            return abort("Failed Cartesian contact→hover at end");

        // // ---------------------------------------------------------- //
        // //  STEP 11 — Return to perception waypoint + verify board detection
        // // ---------------------------------------------------------- //
        // send_feedback(gh, 1.00f, "Returning to perception waypoint");
        // {
        //     // Use ensureBoardDetected so the return also corrects any
        //     // perception misalignment before the next move is planned.
        //     if (!ensureBoardDetected()) {
        //         RCLCPP_WARN(logger, "Board not confirmed after returning — continuing anyway");
        //     }
        // }

   // ---------------------------------------------------------- //
        //  STEP 11 — Return to perception waypoint + verify board detection
        //  Phase 1: Cartesian transit to avoid weird rotations, then
        //           joint-space snap to exact perception angles.
        //  Phase 2: if board not detected, snap to joints again.
        // ---------------------------------------------------------- //
        send_feedback(gh, 1.00f, "Returning to perception waypoint");
        {
            // Phase 1 — Cartesian transit to perception vicinity, then snap to exact joints.
            {
                geometry_msgs::msg::Pose return_pose = fkPose(move_group, PERCEPTION_WAYPOINT);
                return_pose.orientation = ORI_VWALL;
                move_group.setJointValueTarget(PERCEPTION_WAYPOINT);
                if (!moveCartesianSequence(move_group,
                        { move_group.getCurrentPose().pose, return_pose },
                        "perception_waypoint_return_transit", logger)) {
                    RCLCPP_WARN(logger, "Cartesian transit to perception waypoint failed — continuing anyway");
                }
            }
            // Snap to exact joint angles so wrist settles correctly.
            if (!moveToJoints(move_group, PERCEPTION_WAYPOINT,
                              "perception_waypoint_return_snap", logger)) {
                RCLCPP_WARN(logger, "Joint snap to perception waypoint failed — continuing anyway");
            }

            // Phase 2 — check board; if not detected, snap to joints once more.
            {
                std::lock_guard<std::mutex> lock(board_mutex);
                board_detected = -1;
            }
            rclcpp::Rate wait_rate(20);
            int ticks = 0;
            constexpr int MAX_TICKS = 60;   // 3 seconds at 20 Hz
            while (rclcpp::ok() && ticks < MAX_TICKS) {
                {
                    std::lock_guard<std::mutex> lock(board_mutex);
                    if (board_detected != -1) break;
                }
                wait_rate.sleep();
                ++ticks;
            }

            {
                std::lock_guard<std::mutex> lock(board_mutex);
                if (board_detected != 1) {
                    RCLCPP_WARN(logger, "Board not detected after return (value=%d) — retrying joint snap",
                                board_detected.load());
                    // Joint-space retry — re-commands exact angles to correct any remaining drift.
                    if (!moveToJoints(move_group, PERCEPTION_WAYPOINT,
                                      "perception_waypoint_retry_snap", logger)) {
                        RCLCPP_WARN(logger, "Retry joint snap to perception waypoint also failed — continuing anyway");
                    }
                } else {
                    RCLCPP_INFO(logger, "Board confirmed after returning to perception waypoint");
                }
            }
        }
        // ---------------------------------------------------------- //
        //  Advance wall counter AFTER a successful wall move.
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
