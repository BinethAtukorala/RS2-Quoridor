#include <memory>
#include <chrono>
#include <cmath>
#include <string>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

enum class GripperState
{
    IDLE,
    MOVING,        // open / close / drop — target sent directly, wait for joint
    PICKUP_OPEN,   // pickup: open first (direct send)
    PICKUP_PAUSE,  // non-blocking pause between open and close
    PICKUP_CLOSE,  // slow step-by-step close with stall detection
};

class GripperNode : public rclcpp::Node
{
public:
    GripperNode() : Node("gripper_node")
    {
        gripper_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/finger_width_controller/commands", 10);

        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/gripper/status", 10);

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&GripperNode::jointCallback, this, std::placeholders::_1));

        // cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
        //     "/gripper/command", 10,
        //     std::bind(&GripperNode::commandCallback, this, std::placeholders::_1));

         cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/gripper/command",
            rclcpp::QoS(10).transient_local(),
            std::bind(&GripperNode::commandCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            50ms, std::bind(&GripperNode::timerTick, this));

        state_           = GripperState::IDLE;
        current_width_   = 0.04;
        sent_command_    = 0.04;
        target_width_    = 0.04;
        prev_width_      = 0.04;
        stable_count_    = 0;
        pause_ticks_     = 0;
        joint_received_  = false;
        drop_mode_       = false;
        active_object_   = "";

        sensor_tolerance_ = 0.002;  // handles 0.039 vs 0.04 discrepancy
        tolerance_        = 0.005;
        pickup_step_      = 0.001;  // slow step only used during pickup close

        expected_width_["wall"] = 0.017;
        expected_width_["pawn"] = 0.024;

        RCLCPP_INFO(this->get_logger(), "Gripper node ready.");
    }

private:
    // ================================================================
    //  JOINT CALLBACK
    // ================================================================
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            const std::string &n = msg->name[i];
            if (n.find("finger") != std::string::npos ||
                n.find("gripper") != std::string::npos)
            {
                current_width_  = msg->position[i];
                joint_received_ = true;
                return;
            }
        }
        if (!msg->position.empty())
        {
            current_width_  = msg->position[0];
            joint_received_ = true;
        }
    }

    // ================================================================
    //  COMMAND CALLBACK
    // ================================================================
    void commandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (state_ != GripperState::IDLE)
        {
            RCLCPP_WARN(this->get_logger(), "Gripper busy — ignoring: %s", msg->data.c_str());
            return;
        }

        const std::string cmd = msg->data;

        if (cmd == "open")
            startMove(0.04, false);
        else if (cmd == "close")
            startMove(0.0, false);
        else if (cmd.rfind("pickup_", 0) == 0)
        {
            active_object_ = cmd.substr(7);
            startPickup();
        }
        else if (cmd.rfind("drop_", 0) == 0)
        {
            active_object_ = cmd.substr(5);
            startDrop();
        }
        else
            RCLCPP_WARN(this->get_logger(), "Unknown command: %s", cmd.c_str());
    }

    // ================================================================
    //  START HELPERS
    // ================================================================

    // open / close / drop: send target directly, controller handles ramping
    void startMove(double target, bool is_drop)
    {
        target_width_ = target;
        drop_mode_    = is_drop;
        state_        = GripperState::MOVING;
        publishCommand(target_width_);
        RCLCPP_INFO(this->get_logger(), "Moving gripper %.3f -> %.3f",
                    current_width_, target_width_);
    }

    void startPickup()
    {
        RCLCPP_INFO(this->get_logger(), "Pickup: %s", active_object_.c_str());

        if (current_width_ < 0.04 - sensor_tolerance_)
        {
            target_width_ = 0.04;
            state_        = GripperState::PICKUP_OPEN;
            publishCommand(0.04);
            RCLCPP_INFO(this->get_logger(), "Opening before pickup (currently %.3f)...", current_width_);
        }
        else
        {
            beginPickupClose();
        }
    }

    void beginPickupClose()
    {
        sent_command_ = current_width_;
        prev_width_   = current_width_;
        stable_count_ = 0;
        state_        = GripperState::PICKUP_CLOSE;
        RCLCPP_INFO(this->get_logger(), "Pickup: closing slowly from %.3f", current_width_);
    }

    void startDrop()
    {
        if (expected_width_.find(active_object_) == expected_width_.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown object for drop: %s", active_object_.c_str());
            publishStatus("fail");
            return;
        }

        double expected = expected_width_[active_object_];

        // Use loose tolerance (4x) to account for grip drift between pickup and drop
        // e.g. wall stalls at 0.012, drifts to 0.009 — tight 0.005 check would block drop
        if (std::abs(current_width_ - expected) > tolerance_ * 4)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Drop blocked: holding wrong object (%.3f vs %.3f)",
                        current_width_, expected);
            publishStatus("fail");
            return;
        }

        startMove(0.04, true);
    }

    // ================================================================
    //  TIMER TICK
    // ================================================================
    void timerTick()
    {
        if (!joint_received_) return;

        switch (state_)
        {
            case GripperState::IDLE:          break;
            case GripperState::MOVING:        tickMoving();      break;
            case GripperState::PICKUP_OPEN:   tickPickupOpen();  break;
            case GripperState::PICKUP_PAUSE:  tickPause();       break;
            case GripperState::PICKUP_CLOSE:  tickPickupClose(); break;
        }
    }

    // ----------------------------------------------------------------
    //  TICK: MOVING — wait for joint to confirm arrival
    // ----------------------------------------------------------------
    void tickMoving()
    {
        if (std::abs(current_width_ - target_width_) <= sensor_tolerance_)
        {
            RCLCPP_INFO(this->get_logger(), "Gripper reached %.3f (actual %.3f)",
                        target_width_, current_width_);
            state_ = GripperState::IDLE;

            if (drop_mode_)
                RCLCPP_INFO(this->get_logger(), "Drop SUCCESS");

            publishStatus("done");
        }
        // else: wait — command already sent, controller is moving
    }

    // ----------------------------------------------------------------
    //  TICK: PICKUP_OPEN — wait for full open, then pause
    // ----------------------------------------------------------------
    void tickPickupOpen()
    {
        if (std::abs(current_width_ - target_width_) <= sensor_tolerance_)
        {
            RCLCPP_INFO(this->get_logger(), "Opened to %.3f — pausing", current_width_);
            pause_ticks_ = 4;  // 4 x 50ms = 200ms non-blocking
            state_       = GripperState::PICKUP_PAUSE;
        }
        // else: wait — command already sent
    }

    // ----------------------------------------------------------------
    //  TICK: PICKUP_PAUSE
    // ----------------------------------------------------------------
    void tickPause()
    {
        if (--pause_ticks_ <= 0)
            beginPickupClose();
    }

    // ----------------------------------------------------------------
    //  TICK: PICKUP_CLOSE — slow step, stall detection
    //
    //  Stall detection only activates once sent_command_ is below 0.015m.
    //  This prevents false stall triggers from natural deceleration at
    //  the start of closing motion.
    // ----------------------------------------------------------------
    void tickPickupClose()
    {
        bool stall_detection_active = (sent_command_ < 0.015);

        if (stall_detection_active)
        {
            double diff = std::abs(current_width_ - prev_width_);

            if (diff < 0.0005)
                stable_count_++;
            else
                stable_count_ = 0;

            // 5 consecutive ticks (250ms) with no movement = object held or fully closed
            if (stable_count_ >= 5)
            {
                // Hold exactly at current real position — do NOT squeeze further.
                // Controller maintains force here without continuing to close.
                RCLCPP_INFO(this->get_logger(), "Gripper stalled at %.3f — holding", current_width_);
                publishCommand(current_width_);
                sent_command_ = current_width_;
                state_        = GripperState::IDLE;
                evaluatePickup();
                return;
            }
        }
        else
        {
            stable_count_ = 0;
        }

        prev_width_ = current_width_;

        // Send next small closing step
        if (sent_command_ > 0.0)
        {
            sent_command_ = std::max(sent_command_ - pickup_step_, 0.0);
            publishCommand(sent_command_);
        }
        else
        {
            // sent_command_ at 0, no stall — fully closed, nothing grabbed
            RCLCPP_INFO(this->get_logger(), "Gripper fully closed at %.3f", current_width_);
            publishCommand(current_width_);
            sent_command_ = current_width_;
            state_        = GripperState::IDLE;
            evaluatePickup();
        }
    }

    // ----------------------------------------------------------------
    //  PICKUP EVALUATION
    // ----------------------------------------------------------------
    void evaluatePickup()
    {
        double expected = expected_width_[active_object_];

        if (std::abs(current_width_ - expected) <= tolerance_)
        {
            RCLCPP_INFO(this->get_logger(), "Pickup SUCCESS: %.3f (expected %.3f)",
                        current_width_, expected);
            publishStatus("success");
        }
        else if (current_width_ < 0.005 + sensor_tolerance_)
        {
            RCLCPP_ERROR(this->get_logger(), "Pickup FAILED — nothing grasped: %.3f", current_width_);
            publishStatus("fail");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Pickup WRONG object: %.3f (expected %.3f)",
                        current_width_, expected);
            publishStatus("wrong");
        }
    }

    // ================================================================
    //  HELPERS
    // ================================================================
    void publishCommand(double width)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {width};
        gripper_pub_->publish(msg);
    }

    void publishStatus(const std::string &status)
    {
        std_msgs::msg::String msg;
        msg.data = status;
        status_pub_->publish(msg);
    }

    // ================================================================
    //  MEMBERS
    // ================================================================
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            status_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  joint_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr         cmd_sub_;
    rclcpp::TimerBase::SharedPtr                                   timer_;

    GripperState state_;

    double current_width_;   // real value from joint_states
    double sent_command_;    // tracks incremental command during pickup close
    double target_width_;
    double prev_width_;
    int    stable_count_;
    int    pause_ticks_;
    bool   joint_received_;
    bool   drop_mode_;
    std::string active_object_;

    double sensor_tolerance_;
    double tolerance_;
    double pickup_step_;

    std::map<std::string, double> expected_width_;
};

// ================================================================
//  MAIN
// ================================================================
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GripperNode>());
    rclcpp::shutdown();
    return 0;
}