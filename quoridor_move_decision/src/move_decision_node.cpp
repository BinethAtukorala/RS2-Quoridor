#include "quoridor_move_decision/board.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace quoridor {

class MoveDecisionNode : public rclcpp::Node {
 public:
  MoveDecisionNode() : Node("move_decision") {
    max_depth_ = this->declare_parameter<int>("search_depth", 3);

    sub_ = this->create_subscription<std_msgs::msg::String>(
        "/quoridor/compute_move_request", 10,
        std::bind(&MoveDecisionNode::on_request, this, std::placeholders::_1));
    pub_ = this->create_publisher<std_msgs::msg::String>(
        "/quoridor/compute_move_response", 10);

    RCLCPP_INFO(get_logger(), "Move Decision ready (search depth=%d)", max_depth_);
  }

 private:
  void on_request(const std_msgs::msg::String::SharedPtr msg) {
    QuoridorBoard board;
    try {
      board = QuoridorBoard::from_json(msg->data);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to parse board: %s", e.what());
      return;
    }

    RCLCPP_INFO(get_logger(), "Computing best move …");
    auto move = compute_best_move(board);
    if (!move) {
      RCLCPP_ERROR(get_logger(), "No legal moves available!");
      return;
    }

    std_msgs::msg::String out;
    out.data = move->to_json().dump();
    pub_->publish(out);
    RCLCPP_INFO(get_logger(), "Best move: %s", out.data.c_str());
  }

  std::optional<Move> compute_best_move(const QuoridorBoard& board) {
    double best_score = -std::numeric_limits<double>::infinity();
    std::optional<Move> best_move;

    auto moves = ordered_moves(board);
    for (const auto& m : moves) {
      auto child = board.copy();
      child.apply_move(m);
      double score =
          minimax(child, max_depth_ - 1,
                  -std::numeric_limits<double>::infinity(),
                  std::numeric_limits<double>::infinity(), false);
      if (score > best_score) {
        best_score = score;
        best_move = m;
      }
    }
    return best_move;
  }

  double minimax(const QuoridorBoard& board, int depth, double alpha,
                 double beta, bool maximizing) {
    if (depth == 0 || board.game_status() != GameStatus::IN_PROGRESS)
      return evaluate(board);

    auto moves = ordered_moves(board);
    if (moves.empty()) return evaluate(board);

    if (maximizing) {
      double value = -std::numeric_limits<double>::infinity();
      for (const auto& m : moves) {
        auto child = board.copy();
        child.apply_move(m);
        value = std::max(value, minimax(child, depth - 1, alpha, beta, false));
        alpha = std::max(alpha, value);
        if (alpha >= beta) break;
      }
      return value;
    } else {
      double value = std::numeric_limits<double>::infinity();
      for (const auto& m : moves) {
        auto child = board.copy();
        child.apply_move(m);
        value = std::min(value, minimax(child, depth - 1, alpha, beta, true));
        beta = std::min(beta, value);
        if (alpha >= beta) break;
      }
      return value;
    }
  }

  double evaluate(const QuoridorBoard& board) {
    auto status = board.game_status();
    if (status == GameStatus::BOT_WINS) return 1000.0;
    if (status == GameStatus::PLAYER_WINS) return -1000.0;

    auto bot_dist = board.shortest_path_length(board.bot_pos, board.n - 1);
    auto player_dist = board.shortest_path_length(board.player_pos, 0);

    if (!bot_dist) return -500.0;
    if (!player_dist) return 500.0;

    double score = static_cast<double>(*player_dist - *bot_dist);
    score += 0.1 * (board.bot_walls_remaining - board.player_walls_remaining);
    return score;
  }

  std::vector<Move> ordered_moves(const QuoridorBoard& board) {
    auto pawn = board.get_legal_pawn_moves();
    auto walls = board.get_strategic_wall_placements();
    pawn.insert(pawn.end(), walls.begin(), walls.end());
    return pawn;
  }

  int max_depth_{3};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

}  // namespace quoridor

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<quoridor::MoveDecisionNode>());
  rclcpp::shutdown();
  return 0;
}
