#pragma once

#include <nlohmann/json.hpp>

#include <optional>
#include <string>
#include <vector>

namespace quoridor {

enum class Orientation { HORIZONTAL, VERTICAL };
enum class MoveType { PAWN, WALL };
enum class GameStatus { IN_PROGRESS, BOT_WINS, PLAYER_WINS };

struct Pawn {
  int row{0};
  int col{0};

  nlohmann::json to_json() const;
  static Pawn from_json(const nlohmann::json& j);
  bool operator==(const Pawn& o) const { return row == o.row && col == o.col; }
};

struct Wall {
  int row{0};
  int col{0};
  Orientation orientation{Orientation::HORIZONTAL};

  nlohmann::json to_json() const;
  static Wall from_json(const nlohmann::json& j);
};

struct Move {
  MoveType move_type{MoveType::PAWN};
  std::optional<Pawn> target;
  std::optional<Wall> wall;

  nlohmann::json to_json() const;
};

class QuoridorBoard {
 public:
  int n{5};
  Pawn bot_pos;
  Pawn player_pos;
  int bot_walls_remaining{4};
  int player_walls_remaining{4};
  std::vector<Wall> walls;
  std::string turn{"bot"};

  static QuoridorBoard from_json(const std::string& s);

  QuoridorBoard copy() const { return *this; }
  GameStatus game_status() const;

  std::optional<int> shortest_path_length(const Pawn& start, int goal_row) const;

  std::vector<Move> get_legal_pawn_moves() const;
  std::vector<Move> get_strategic_wall_placements() const;

  bool apply_move(const Move& m);
};

}  // namespace quoridor
