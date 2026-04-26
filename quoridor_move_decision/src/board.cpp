#include "quoridor_move_decision/board.hpp"

#include <stdexcept>

namespace quoridor {

nlohmann::json Pawn::to_json() const {
  return nlohmann::json{{"row", row}, {"col", col}};
}

Pawn Pawn::from_json(const nlohmann::json& j) {
  return Pawn{j.at("row").get<int>(), j.at("col").get<int>()};
}

nlohmann::json Wall::to_json() const {
  return nlohmann::json{
      {"row", row},
      {"col", col},
      {"orientation",
       orientation == Orientation::HORIZONTAL ? "HORIZONTAL" : "VERTICAL"}};
}

Wall Wall::from_json(const nlohmann::json& j) {
  Wall w;
  w.row = j.at("row").get<int>();
  w.col = j.at("col").get<int>();
  w.orientation = j.at("orientation").get<std::string>() == "HORIZONTAL"
                      ? Orientation::HORIZONTAL
                      : Orientation::VERTICAL;
  return w;
}

nlohmann::json Move::to_json() const {
  nlohmann::json j;
  j["move_type"] = move_type == MoveType::PAWN ? "PAWN" : "WALL";
  if (target) j["target"] = target->to_json();
  if (wall) j["wall"] = wall->to_json();
  return j;
}

QuoridorBoard QuoridorBoard::from_json(const std::string& s) {
  auto j = nlohmann::json::parse(s);
  QuoridorBoard b;
  b.n = j.value("n", 5);
  b.bot_pos = Pawn::from_json(j.at("bot_pos"));
  b.player_pos = Pawn::from_json(j.at("player_pos"));
  b.bot_walls_remaining = j.value("bot_walls_remaining", 4);
  b.player_walls_remaining = j.value("player_walls_remaining", 4);
  b.turn = j.value("turn", std::string{"bot"});
  if (j.contains("walls")) {
    for (const auto& w : j.at("walls")) b.walls.push_back(Wall::from_json(w));
  }
  return b;
}

GameStatus QuoridorBoard::game_status() const {
  if (bot_pos.row == n - 1) return GameStatus::BOT_WINS;
  if (player_pos.row == 0) return GameStatus::PLAYER_WINS;
  return GameStatus::IN_PROGRESS;
}

// TODO: port from quoridor_utils.py::shortest_path_length (BFS respecting walls).
std::optional<int> QuoridorBoard::shortest_path_length(const Pawn&, int) const {
  throw std::logic_error("shortest_path_length not implemented");
}

// TODO: port from quoridor_utils.py::get_legal_pawn_moves (includes jump logic).
std::vector<Move> QuoridorBoard::get_legal_pawn_moves() const {
  throw std::logic_error("get_legal_pawn_moves not implemented");
}

// TODO: port from quoridor_utils.py::get_strategic_wall_placements.
std::vector<Move> QuoridorBoard::get_strategic_wall_placements() const {
  throw std::logic_error("get_strategic_wall_placements not implemented");
}

// TODO: port from quoridor_utils.py::apply_move.
bool QuoridorBoard::apply_move(const Move&) {
  throw std::logic_error("apply_move not implemented");
}

}  // namespace quoridor
