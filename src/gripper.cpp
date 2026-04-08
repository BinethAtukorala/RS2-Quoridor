#include <memory>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

class GripperNode : public rclcpp::Node
{
public:
    GripperNode() : Node("gripper_node")
    {
        // Publisher to gripper controller
        gripper_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/finger_width_controller/commands", 10
        );

        // Status publisher (for control node)
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/gripper/status", 10
        );

        // Joint state subscriber
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&GripperNode::jointCallback, this, std::placeholders::_1)
        );

        // Command subscriber
        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/gripper/command", 10,
            std::bind(&GripperNode::commandCallback, this, std::placeholders::_1)
        );

        current_width_ = 0.04;   // assume starts open
        last_joint_angle_ = 0.0;

        // Expected widths
        expected_width_["wall"] = 0.03;
        expected_width_["pawn"] = 0.022;

        tolerance_ = 0.005;
    }

private:
    // -------------------- JOINT CALLBACK --------------------
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!msg->position.empty())
        {
            last_joint_angle_ = msg->position[0];
        }
    }

    // -------------------- COMMAND CALLBACK --------------------
    void commandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string cmd = msg->data;

        if (cmd == "open")
        {
            moveGripper(0.04);
            publishStatus("done");
        }
        else if (cmd == "close")
        {
            moveGripper(0.0);
            publishStatus("done");
        }
        else if (cmd.find("pickup_") == 0)
        {
            std::string obj = cmd.substr(7);
            pickupObject(obj);
        }
        else if (cmd.find("drop_") == 0)
        {
            std::string obj = cmd.substr(5);
            dropObject(obj);
        }
    }

    // -------------------- MOVE GRIPPER --------------------
    void moveGripper(double target_width)
    {
        std_msgs::msg::Float64MultiArray msg;
        double step = 0.002;

        if (current_width_ < target_width)
        {
            for (double w = current_width_; w <= target_width; w += step)
            {
                msg.data = {w};
                gripper_pub_->publish(msg);
                rclcpp::sleep_for(20ms);
            }
        }
        else
        {
            for (double w = current_width_; w >= target_width; w -= step)
            {
                msg.data = {w};
                gripper_pub_->publish(msg);
                rclcpp::sleep_for(20ms);
            }
        }

        msg.data = {target_width};
        gripper_pub_->publish(msg);

        current_width_ = target_width;
    }

    // -------------------- PICKUP --------------------
    void pickupObject(const std::string &obj)
    {
        RCLCPP_INFO(this->get_logger(), "Pickup: %s", obj.c_str());

        // Ensure open
        if (current_width_ < 0.04)
        {
            moveGripper(0.04);
            rclcpp::sleep_for(200ms);
        }

        // Close gradually while monitoring joint
        std_msgs::msg::Float64MultiArray msg;
        double step = 0.001;
        double prev_angle = last_joint_angle_;

        while (true)
        {
            current_width_ -= step;
            if (current_width_ < 0.0)
                current_width_ = 0.0;

            msg.data = {current_width_};
            gripper_pub_->publish(msg);

            rclcpp::sleep_for(20ms);

            double diff = std::abs(last_joint_angle_ - prev_angle);
            if (diff < 0.0005)
                break;

            prev_angle = last_joint_angle_;
        }

        // Evaluate result
        double expected = expected_width_[obj];

        if (std::abs(current_width_ - expected) <= tolerance_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Pickup SUCCESS (correct): %.3f", current_width_);
            publishStatus("success");
        }
        else if (current_width_ < 0.005)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Pickup FAILED (nothing): %.3f", current_width_);
            publishStatus("fail");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                        "Pickup WRONG object: %.3f (expected %.3f)",
                        current_width_, expected);
            publishStatus("wrong");
        }
    }

    // -------------------- DROP --------------------
    void dropObject(const std::string &obj)
    {
        RCLCPP_INFO(this->get_logger(), "Drop: %s", obj.c_str());

        double expected = expected_width_[obj];

        if (std::abs(current_width_ - expected) > tolerance_)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Drop blocked: holding wrong object (%.3f vs %.3f)",
                        current_width_, expected);
            publishStatus("fail");
            return;
        }

        moveGripper(0.04);

        RCLCPP_INFO(this->get_logger(), "Drop SUCCESS");
        publishStatus("done");
    }

    // -------------------- STATUS --------------------
    void publishStatus(const std::string &status)
    {
        std_msgs::msg::String msg;
        msg.data = status;
        status_pub_->publish(msg);
    }

    // -------------------- VARIABLES --------------------
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;

    double current_width_;
    double last_joint_angle_;

    std::map<std::string, double> expected_width_;
    double tolerance_;
};

// -------------------- MAIN --------------------
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}