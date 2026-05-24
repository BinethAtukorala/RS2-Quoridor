#include <memory>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <chrono>
#include <future>
#include <set>
#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "quoridor_interfaces/action/bot_move.hpp"

static const std::string POSES_FILE = "/rs2_ws/src/perception/poses.txt";

static const double ORI_X = 1.0, ORI_Y = 0.0, ORI_Z = -0.0000032, ORI_W = 0.0000159;

using BotMove    = quoridor_interfaces::action::BotMove;
using GoalHandle = rclcpp_action::ClientGoalHandle<BotMove>;

struct GridPos  { int row, col; };
struct Position { double x, y, z; };

struct PawnCell {
    GridPos grid;
    Position pos;
};

struct WallCell {
    GridPos grid;
    Position pos;
};

// ---------------------------------------------------------------------------
// Valid pawn destinations from (r, c) on a 5x5 board
// Includes: orthogonal adjacents, orthogonal jumps (opponent jump), diagonals
// ---------------------------------------------------------------------------
std::vector<GridPos> pawnMoves(int r, int c)
{
    std::vector<GridPos> moves;
    const int N = 5;

    // orthogonal: adjacent and jump (2 squares)
    int dirs[4][2] = {{0,1},{0,-1},{1,0},{-1,0}};
    for (auto &d : dirs) {
        int nr1 = r + d[0], nc1 = c + d[1];
        if (nr1 >= 0 && nr1 < N && nc1 >= 0 && nc1 < N)
            moves.push_back({nr1, nc1});
        int nr2 = r + 2*d[0], nc2 = c + 2*d[1];
        if (nr2 >= 0 && nr2 < N && nc2 >= 0 && nc2 < N)
            moves.push_back({nr2, nc2});
    }

    // diagonals
    int diags[4][2] = {{1,1},{1,-1},{-1,1},{-1,-1}};
    for (auto &d : diags) {
        int nr = r + d[0], nc = c + d[1];
        if (nr >= 0 && nr < N && nc >= 0 && nc < N)
            moves.push_back({nr, nc});
    }

    return moves;
}

// ---------------------------------------------------------------------------
// File loading
// ---------------------------------------------------------------------------
bool loadPosesFile(const std::string &path, rclcpp::Logger logger,
                   std::vector<PawnCell> &pawns, std::vector<WallCell> &walls)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        RCLCPP_ERROR(logger, "Cannot open poses file: %s", path.c_str());
        return false;
    }

    std::string line;
    int line_num = 0;
    int count = 0;

    while (std::getline(file, line)) {
        ++line_num;
        if (line.empty() || line[0] == '#') continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream ss(line);

        int row, col;
        double x, y, z;
        if (!(ss >> row >> col >> x >> y >> z)) {
            RCLCPP_WARN(logger, "Skipping malformed line %d: '%s'", line_num, line.c_str());
            continue;
        }

        if (count < 25) {
            pawns.push_back({{row, col}, {x, y, z}});
        } else {
            walls.push_back({{row, col}, {x, y, z}});
        }
        ++count;
    }

    RCLCPP_INFO(logger, "Loaded %zu pawn cells, %zu wall cells", pawns.size(), walls.size());
    return true;
}

// ---------------------------------------------------------------------------
// Goal builders
// ---------------------------------------------------------------------------
BotMove::Goal makePawnGoal(const Position &start, const Position &end)
{
    BotMove::Goal goal;
    goal.piece_type = "p";

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

BotMove::Goal makeWallGoal(const Position &end, const std::string &type)
{
    BotMove::Goal goal;
    goal.piece_type = type;  // "h" or "v"

    // dummy start — overridden in control.cpp
    goal.start.position.x    = 0.0;
    goal.start.position.y    = 0.0;
    goal.start.position.z    = 0.0;
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

// ---------------------------------------------------------------------------
// Node
// ---------------------------------------------------------------------------
class QuoridorPoseTester : public rclcpp::Node
{
public:
    QuoridorPoseTester()
    : Node("quoridor_pose_tester")
    {
        client_ = rclcpp_action::create_client<BotMove>(this, "/quoridor/bot_execute");
    }

    void run()
    {
        auto logger = get_logger();

        std::vector<PawnCell> pawns;
        std::vector<WallCell> walls;
        if (!loadPosesFile(POSES_FILE, logger, pawns, walls)) return;

        RCLCPP_INFO(logger, "Waiting for /quoridor/bot_execute ...");
        if (!client_->wait_for_action_server(std::chrono::seconds(30))) {
            RCLCPP_ERROR(logger, "Action server not available after 30s. Aborting");
            return;
        }

        // Build lookup: grid -> Position for pawns
        std::map<std::pair<int,int>, Position> pawnMap;
        for (auto &c : pawns)
            pawnMap[{c.grid.row, c.grid.col}] = c.pos;

        // ----------------------------------------------------------------
        // PAWN TESTS
        // For every cell, send a goal to every valid destination
        // ----------------------------------------------------------------
        int total_pawn = 0, pass_pawn = 0, fail_pawn = 0;
        RCLCPP_INFO(logger, "=== PAWN MOVE TESTS ===");

        for (auto &cell : pawns)
        {
            auto dests = pawnMoves(cell.grid.row, cell.grid.col);
            for (auto &dest : dests)
            {
                auto it = pawnMap.find({dest.row, dest.col});
                if (it == pawnMap.end()) continue;  // destination not in file

                ++total_pawn;
                RCLCPP_INFO(logger, "[Pawn #%d] (%d,%d) -> (%d,%d)",
                            total_pawn + 1, cell.grid.row, cell.grid.col, dest.row, dest.col);

                bool ok = sendAndWait(makePawnGoal(cell.pos, it->second), total_pawn);
                if (ok) {
                    ++pass_pawn;
                    RCLCPP_INFO(logger, "  PASS");
                } else {
                    ++fail_pawn;
                    RCLCPP_WARN(logger, "  FAIL (%d,%d)->(%d,%d)",
                                cell.grid.row, cell.grid.col, dest.row, dest.col);
                }
            }
        }

        RCLCPP_INFO(logger, "=== PAWN RESULTS: %d/%d passed, %d failed ===",
                    pass_pawn, total_pawn, fail_pawn);

        // ----------------------------------------------------------------
        // WALL TESTS
        // For each of the 16 wall grid positions:
        //   place all 4 walls horizontally, then all 4 walls vertically
        // ----------------------------------------------------------------
        int total_wall = 0, pass_wall = 0, fail_wall = 0;
        RCLCPP_INFO(logger, "=== WALL PLACEMENT TESTS ===");

        for (auto &cell : walls)
        {
            for (const std::string &type : {"h", "v"})
            {
                RCLCPP_INFO(logger, "[Wall %s] grid (%d,%d) — placing all 4 walls",
                            type.c_str(), cell.grid.row, cell.grid.col);

                for (int w = 0; w < 4; ++w)
                {
                    ++total_wall;
                    RCLCPP_INFO(logger, "  [Wall #%d] wall %d/4 grid (%d,%d) type=%s",
                        total_wall + 1, w + 1, cell.grid.row, cell.grid.col, type.c_str());

                    bool ok = sendAndWait(makeWallGoal(cell.pos, type), total_wall);
                    if (ok) {
                        ++pass_wall;
                        RCLCPP_INFO(logger, "  PASS");
                    } else {
                        ++fail_wall;
                        RCLCPP_WARN(logger, "  FAIL wall %d at (%d,%d) type=%s",
                                    w + 1, cell.grid.row, cell.grid.col, type.c_str());
                    }
                }
            }
        }

        RCLCPP_INFO(logger, "=== WALL RESULTS: %d/%d passed, %d failed ===",
                    pass_wall, total_wall, fail_wall);

        // ----------------------------------------------------------------
        // Summary
        // ----------------------------------------------------------------
        RCLCPP_INFO(logger, "==============================");
        RCLCPP_INFO(logger, "TOTAL PAWN : %d/%d passed", pass_pawn, total_pawn);
        RCLCPP_INFO(logger, "TOTAL WALL : %d/%d passed", pass_wall, total_wall);
        RCLCPP_INFO(logger, "OVERALL    : %d/%d passed",
                    pass_pawn + pass_wall, total_pawn + total_wall);
        RCLCPP_INFO(logger, "==============================");
    }

private:
    rclcpp_action::Client<BotMove>::SharedPtr client_;

    bool sendAndWait(const BotMove::Goal &goal, int idx)
    {
        auto logger = get_logger();

        std::promise<bool> result_promise;
        std::shared_future<bool> result_future = result_promise.get_future().share();

        auto opts = rclcpp_action::Client<BotMove>::SendGoalOptions();

        opts.feedback_callback =
            [this, idx](GoalHandle::SharedPtr,
                        const std::shared_ptr<const BotMove::Feedback> fb)
            {
                RCLCPP_INFO(get_logger(), "  [%d] %.0f%%", idx, fb->progress * 100.0f);
            };

        opts.result_callback =
            [this, idx, &result_promise](const GoalHandle::WrappedResult &wr)
            {
                switch (wr.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        result_promise.set_value(wr.result->result);
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(get_logger(), "  [%d] ABORTED", idx);
                        result_promise.set_value(false);
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(get_logger(), "  [%d] CANCELED", idx);
                        result_promise.set_value(false);
                        break;
                    default:
                        RCLCPP_ERROR(get_logger(), "  [%d] unknown result", idx);
                        result_promise.set_value(false);
                        break;
                }
            };

        auto gh_future = client_->async_send_goal(goal, opts);
        if (rclcpp::spin_until_future_complete(
                shared_from_this(), gh_future, std::chrono::seconds(10))
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(logger, "  [%d] timed out on goal acceptance", idx);
            return false;
        }

        auto gh = gh_future.get();
        if (!gh) {
            RCLCPP_ERROR(logger, "  [%d] goal REJECTED", idx);
            return false;
        }

        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future)
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(logger, "  [%d] error waiting for result", idx);
            return false;
        }

        return result_future.get();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QuoridorPoseTester>();
    node->run();
    rclcpp::shutdown();
    return 0;
}