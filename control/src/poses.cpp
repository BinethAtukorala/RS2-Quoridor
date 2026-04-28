#include <memory>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <chrono>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "quoridor_interfaces/action/bot_move.hpp"

static const std::string POSES_FILE = "/home/bihan/rs2_ws/src/perception/poses.txt";
static const std::string PIECE_TYPE = "p";

static const double ORI_X = 1.0, ORI_Y = 0.0, ORI_Z = -0.0000032, ORI_W = 0.0000159;

using BotMove    = quoridor_interfaces::action::BotMove;
using GoalHandle = rclcpp_action::ClientGoalHandle<BotMove>;

struct Position { double x, y, z; };

std::vector<Position> loadPoses(const std::string &path, rclcpp::Logger logger)
{
    std::vector<Position> poses;
    std::ifstream file(path);

    if (!file.is_open()) {
        RCLCPP_ERROR(logger, "Cannot open poses file: %s", path.c_str());
        return poses;
    }

    std::string line;
    int line_num = 0;
    while (std::getline(file, line)) {
        ++line_num;
        if (line.empty() || line[0] == '#') continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream ss(line);
        Position p;
        if (!(ss >> p.x >> p.y >> p.z)) {
            RCLCPP_WARN(logger, "Skipping malformed line %d: '%s'", line_num, line.c_str());
            continue;
        }
        poses.push_back(p);
    }

    RCLCPP_INFO(logger, "Loaded %zu positions from %s", poses.size(), path.c_str());
    return poses;
}

BotMove::Goal makeGoal(const Position &start, const Position &end)
{
    BotMove::Goal goal;
    goal.piece_type = PIECE_TYPE;

    goal.start.position.x    = start.x;
    goal.start.position.y    = start.y;
    goal.start.position.z    = start.z;
    goal.start.orientation.x = ORI_X;
    goal.start.orientation.y = ORI_Y;
    goal.start.orientation.z = ORI_Z;
    goal.start.orientation.w = ORI_W;

    goal.end.position.x    = end.x;
    goal.end.position.y    = end.y;
    goal.end.position.z    = end.z;
    goal.end.orientation.x = ORI_X;
    goal.end.orientation.y = ORI_Y;
    goal.end.orientation.z = ORI_Z;
    goal.end.orientation.w = ORI_W;

    return goal;
}

class PoseSequencer : public rclcpp::Node
{
public:
    PoseSequencer()
    : Node("pose_sequencer")
    {
        client_ = rclcpp_action::create_client<BotMove>(this, "/quoridor/bot_execute");
    }

    void run()
    {
        auto logger = get_logger();

        std::vector<Position> positions = loadPoses(POSES_FILE, logger);
        if (positions.size() < 2) {
            RCLCPP_ERROR(logger, "Need at least 2 positions, aborting");
            return;
        }

        const std::size_t num_moves = positions.size() - 1;
        RCLCPP_INFO(logger, "%zu positions loaded. %zu moves to execute",
                    positions.size(), num_moves);

        RCLCPP_INFO(logger, "Waiting for /quoridor/bot_execute ...");
        if (!client_->wait_for_action_server(std::chrono::seconds(30))) {
            RCLCPP_ERROR(logger, "Action server os not available after 30 seconds. Aborting");
            return;
        }

        for (std::size_t i = 0; i < num_moves; ++i)
        {
            const Position &start = positions[i];
            const Position &end   = positions[i + 1];

            RCLCPP_INFO(logger,
                        "Move %zu/%zu | (%.4f, %.4f, %.4f) → (%.4f, %.4f, %.4f)",
                        i + 1, num_moves,
                        start.x, start.y, start.z,
                        end.x,   end.y,   end.z);

            if (!sendAndWait(makeGoal(start, end), i + 1)) {
                RCLCPP_ERROR(logger, "Move %zu FAILED - stopping", i + 1);
                return;
            }

            RCLCPP_INFO(logger, "Move %zu complete", i + 1);
        }

        RCLCPP_INFO(logger, "All %zu moves completed successfully", num_moves);
    }

private:
    rclcpp_action::Client<BotMove>::SharedPtr client_;

    bool sendAndWait(const BotMove::Goal &goal, std::size_t idx)
    {
        auto logger = get_logger();

        std::promise<bool> result_promise;
        std::shared_future<bool> result_future = result_promise.get_future().share();

        auto opts = rclcpp_action::Client<BotMove>::SendGoalOptions();

        opts.feedback_callback =
            [this, idx](GoalHandle::SharedPtr,
                        const std::shared_ptr<const BotMove::Feedback> fb)
            {
                RCLCPP_INFO(get_logger(), "  [move %zu] %.0f%%",
                            idx, fb->progress * 100.0f);
            };

        opts.result_callback =
            [this, idx, &result_promise](const GoalHandle::WrappedResult &wr)
            {
                switch (wr.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(get_logger(), "  [Move %zu] SUCCEEDED", idx);
                        result_promise.set_value(wr.result->result);
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(get_logger(), "  [Move %zu] ABORTED", idx);
                        result_promise.set_value(false);
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(get_logger(), "  [Move %zu] CANCELED", idx);
                        result_promise.set_value(false);
                        break;
                    default:
                        RCLCPP_ERROR(get_logger(), "  [Move %zu] unknown result", idx);
                        result_promise.set_value(false);
                        break;
                }
            };

        auto gh_future = client_->async_send_goal(goal, opts);
        if (rclcpp::spin_until_future_complete(
                shared_from_this(), gh_future, std::chrono::seconds(10))
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(logger, "  [Move %zu] timed out on goal acceptance", idx);
            return false;
        }

        auto gh = gh_future.get();
        if (!gh) {
            RCLCPP_ERROR(logger, "  [Move %zu] goal REJECTED by server", idx);
            return false;
        }

        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future)
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(logger, "  [Move %zu] error waiting for result", idx);
            return false;
        }

        return result_future.get();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseSequencer>();
    node->run();
    rclcpp::shutdown();
    return 0;
}