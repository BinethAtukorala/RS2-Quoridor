import json

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import Pose
from threading import Lock
import numpy as np

from quoridor_interfaces.action import BotMove as BotMoveAction
from quoridor_interfaces.srv import GetCoords

from quoridor_game.quoridor_utils import (
    Move,
    MoveType,
    Orientation,
    Pawn,
    QuoridorBoard,
    Wall,
)


class StateManager(Node):

    def __init__(self):
        super().__init__('state_manager')

        self.board = QuoridorBoard()
        # Perception state: single lock guards both grids + last-applied snapshots
        self.perception_lock = Lock()
        self.latest_pawn_grid = None
        self.latest_wall_grid = None
        self.last_applied_pawn_grid = None
        self.last_applied_wall_grid = None
        # Snapshot of last_applied_* from before the most recent programmatic
        # move. Used to silently absorb perception frames that still show the
        # pre-move state while the robot is physically executing.
        self.prev_applied_pawn_grid = None
        self.prev_applied_wall_grid = None
        self.input_mode = "manual"      # "manual" | "perception"
        self.bot_thinking = False

        # --- publishers ---
        self.pub_board_state = self.create_publisher(
            String, '/quoridor/board_state', 10)
        self.pub_compute_request = self.create_publisher(
            String, '/quoridor/compute_move_request', 10)
        # Move execution subsystem — action client. The action server is
        # expected to advertise BotMove on /quoridor/bot_execute.
        self.bot_execute_client = ActionClient(
            self, BotMoveAction, '/quoridor/bot_execute')
        self.bot_executing = False

        # --- subscribers ---
        self.sub_player_move = self.create_subscription(
            String, '/quoridor/player_move', self.on_player_move, 10)
        self.sub_compute_response = self.create_subscription(
            String, '/quoridor/compute_move_response', self.on_bot_move_computed, 10)
        self.sub_game_command = self.create_subscription(
            String, '/quoridor/game_command', self.on_game_command, 10)
        
        # PERCEPTION SUBSYSTEM
        self.sub_board_perception = self.create_subscription(
            Int32MultiArray, '/perception/board_state', self.on_board_update, 10)
        self.sub_wall_perception = self.create_subscription(
            Int32MultiArray, '/perception/wall_state', self.on_wall_update, 10)

        # 3D coordinate caches keyed by perception (row, col). Populated from
        # live topics (only detected pieces) and from the file-backed
        # /get_pawns and /get_walls services (all calibrated grid cells) as a
        # fallback for cells that have not been observed yet.
        self.coords_lock = Lock()
        self.pawn_coords_3d: dict[tuple[int, int], tuple[float, float, float]] = {}
        self.wall_coords_3d: dict[tuple[int, int], tuple[float, float, float]] = {}

        self.sub_pawns_3d = self.create_subscription(
            Float32MultiArray, '/perception/pawns_3d', self.on_pawns_3d, 10)
        self.sub_walls_3d = self.create_subscription(
            Float32MultiArray, '/perception/walls_inside_3d', self.on_walls_3d, 10)

        self.get_pawns_client = self.create_client(GetCoords, '/get_pawns')
        self.get_walls_client = self.create_client(GetCoords, '/get_walls')
        self._seed_coords_from_services()

        self.get_logger().info('State Manager initialised -- waiting for "start" command')
        self.publish_board_state()

    # ------------------------------------------------------------------ #
    #  Publishing                                                         #
    # ------------------------------------------------------------------ #

    def publish_board_state(self):
        state = self.board.to_dict()
        state["input_mode"] = self.input_mode
        state["bot_thinking"] = self.bot_thinking
        state["bot_executing"] = self.bot_executing
        msg = String()
        msg.data = json.dumps(state)
        self.pub_board_state.publish(msg)

    # ------------------------------------------------------------------ #
    #  Player move handling                                               #
    # ------------------------------------------------------------------ #

    def on_player_move(self, msg: String):
        if self.board.game_status != "in_progress":
            self.get_logger().warn('Game is not in progress -- move ignored')
            return
        if self.bot_executing:
            self.get_logger().warn('Bot is still executing -- player move ignored')
            return
        if self.board.current_turn != "player":
            self.get_logger().warn('Not the player\'s turn -- move ignored')
            return

        try:
            move = Move.from_dict(json.loads(msg.data))
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f'Bad player move payload: {e}')
            return

        if not self.board.apply_move(move):
            self.get_logger().warn('Illegal player move rejected')
            return

        self._sync_snapshots_to_board()
        self.get_logger().info(f'Player move applied: {msg.data}')
        self.publish_board_state()

        if self.board.game_status != "in_progress":
            self.get_logger().info(f'Game over -- {self.board.game_status}')
            return

        self.request_bot_move()

    # ------------------------------------------------------------------ #
    #  Bot move handling                                                  #
    # ------------------------------------------------------------------ #

    def request_bot_move(self):
        self.bot_thinking = True
        self.publish_board_state()
        req = String()
        req.data = self.board.to_json()
        self.pub_compute_request.publish(req)
        self.get_logger().info('Requested bot move from Move Decision')

    def on_bot_move_computed(self, msg: String):
        self.bot_thinking = False
        try:
            move = Move.from_dict(json.loads(msg.data))
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f'Bad bot move payload: {e}')
            return

        # Capture pre-move bot position for the action goal's start pose.
        prev_bot_pos = Pawn(self.board.bot_pos_.x, self.board.bot_pos_.y)

        if not self.board.apply_move(move):
            self.get_logger().error('Move Decision returned an illegal move!')
            self.publish_board_state()
            return

        self._sync_snapshots_to_board()
        self.get_logger().info(f'Bot move applied: {msg.data}')

        self.publish_board_state()

        self._send_bot_execute_goal(move, prev_bot_pos)

        if self.board.game_status != "in_progress":
            self.get_logger().info(f'Game over -- {self.board.game_status}')

    # ------------------------------------------------------------------ #
    #  Bot execute action                                                 #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _pose_from_xyz(xyz: tuple[float, float, float]) -> Pose:
        p = Pose()
        p.position.x = float(xyz[0])
        p.position.y = float(xyz[1])
        p.position.z = float(xyz[2])
        p.orientation.w = 1.0
        return p

    def _board_to_perception_rc(self, bx: int, by: int, grid_n: int) -> tuple[int, int]:
        # state_manager flips incoming grids with np.flipud, so board y = row
        # in the flipped grid => raw perception row = (grid_n - 1) - by.
        return (grid_n - 1 - by, bx)

    def _lookup_pawn_3d(self, bx: int, by: int):
        rc = self._board_to_perception_rc(bx, by, self.board.n_)
        with self.coords_lock:
            return self.pawn_coords_3d.get(rc)

    def _lookup_wall_3d(self, bx: int, by: int):
        rc = self._board_to_perception_rc(bx, by, self.board.wall_n_)
        with self.coords_lock:
            return self.wall_coords_3d.get(rc)

    def _send_bot_execute_goal(self, move: Move, prev_bot_pos: Pawn):
        goal = BotMoveAction.Goal()
        if move.move_type == MoveType.PAWN:
            start_xyz = self._lookup_pawn_3d(prev_bot_pos.x, prev_bot_pos.y)
            end_xyz = self._lookup_pawn_3d(move.target.x, move.target.y)
            if start_xyz is None:
                self.get_logger().error(
                    f'No 3D coord for pawn start ({prev_bot_pos.x},{prev_bot_pos.y}) '
                    '-- cannot send bot_execute goal')
                return
            if end_xyz is None:
                self.get_logger().error(
                    f'No 3D coord for pawn end ({move.target.x},{move.target.y}) '
                    '-- cannot send bot_execute goal')
                return
            goal.piece_type = 'p'
            goal.start = self._pose_from_xyz(start_xyz)
            goal.end = self._pose_from_xyz(end_xyz)
        else:
            wx, wy = move.wall.pos
            end_xyz = self._lookup_wall_3d(wx, wy)
            if end_xyz is None:
                self.get_logger().error(
                    f'No 3D coord for wall slot ({wx},{wy}) '
                    '-- cannot send bot_execute goal')
                return
            goal.piece_type = 'v' if move.wall.orientation == Orientation.VER else 'h'
            # Control ignores goal.start for walls (uses a fixed pickup joint
            # config), but we still send the target pose for both fields.
            goal.start = self._pose_from_xyz(end_xyz)
            goal.end = self._pose_from_xyz(end_xyz)

        if not self.bot_execute_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                '/quoridor/bot_execute action server not available -- skipping execution')
            return

        self.bot_executing = True
        self.get_logger().info(
            f'Sending bot execute goal: piece={goal.piece_type} '
            f'start=({goal.start.position.x},{goal.start.position.y}) '
            f'end=({goal.end.position.x},{goal.end.position.y})')
        send_future = self.bot_execute_client.send_goal_async(
            goal, feedback_callback=self._on_bot_execute_feedback)
        send_future.add_done_callback(self._on_bot_execute_goal_response)

    def _on_bot_execute_feedback(self, feedback_msg):
        self.get_logger().debug(
            f'Bot execute progress: {feedback_msg.feedback.progress:.2f}')

    def _on_bot_execute_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Bot execute goal rejected by server')
            self.bot_executing = False
            return
        self.get_logger().info('Bot execute goal accepted, awaiting result')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_bot_execute_result)

    def _on_bot_execute_result(self, future):
        self.bot_executing = False
        result = future.result().result
        if result.result:
            self.get_logger().info('Bot execute completed successfully')
        else:
            self.get_logger().error('Bot execute reported failure')
        self.publish_board_state()

    # ------------------------------------------------------------------ #
    #  Game commands                                                      #
    # ------------------------------------------------------------------ #

    def on_game_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        command = cmd.get("command", "")

        if command == "start":
            bot_first = cmd.get("bot_first", False)
            self.board = QuoridorBoard()
            self.board.current_turn = "bot" if bot_first else "player"
            self.bot_thinking = False
            self._reset_perception_snapshots()
            self.get_logger().info(
                f'New game started -- {"bot" if bot_first else "player"} goes first')
            self.publish_board_state()
            if bot_first:
                self.request_bot_move()

        elif command == "toggle_input":
            self.input_mode = (
                "perception" if self.input_mode == "manual" else "manual"
            )
            if self.input_mode == "perception":
                self._reset_perception_snapshots()
            self.get_logger().info(f'Input mode: {self.input_mode}')
            self.publish_board_state()

        elif command == "estop":
            self.board.game_status = "stopped"
            self.bot_thinking = False
            self.get_logger().warn('Emergency stop -- game halted')
            self.publish_board_state()

    # ------------------------------------------------------------------ #
    #  Perception                                                         #
    # ------------------------------------------------------------------ #

    def _ingest_coords_flat(self, target: dict, data, label: str):
        if len(data) % 5 != 0:
            self.get_logger().warn(
                f'{label} payload length {len(data)} is not a multiple of 5 -- ignoring')
            return
        with self.coords_lock:
            for i in range(0, len(data), 5):
                r = int(round(float(data[i])))
                c = int(round(float(data[i + 1])))
                x = float(data[i + 2])
                y = float(data[i + 3])
                z = float(data[i + 4])
                target[(r, c)] = (x, y, z)

    def on_pawns_3d(self, msg: Float32MultiArray):
        self._ingest_coords_flat(self.pawn_coords_3d, msg.data, 'pawns_3d')

    def on_walls_3d(self, msg: Float32MultiArray):
        self._ingest_coords_flat(self.wall_coords_3d, msg.data, 'walls_inside_3d')

    def _seed_coords_from_services(self, timeout_sec: float = 3.0):
        for client, target, label in (
            (self.get_pawns_client, self.pawn_coords_3d, '/get_pawns'),
            (self.get_walls_client, self.wall_coords_3d, '/get_walls'),
        ):
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().warn(
                    f'{label} service unavailable -- 3D coord fallback not seeded')
                continue
            future = client.call_async(GetCoords.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            if not future.done() or future.result() is None:
                self.get_logger().warn(f'{label} service call timed out')
                continue
            self._ingest_coords_flat(target, future.result().data, label)
            self.get_logger().info(
                f'{label} seeded {len(target)} grid cells')

    def _reset_perception_snapshots(self):
        with self.perception_lock:
            self.latest_pawn_grid = None
            self.latest_wall_grid = None
            self.last_applied_pawn_grid = None
            self.last_applied_wall_grid = None
            self.prev_applied_pawn_grid = None
            self.prev_applied_wall_grid = None

    def _synthesize_grids_from_board(self):
        """Build pawn/wall grids that match what perception would publish for
        the current board state (already in board-coord orientation)."""
        n = self.board.n_
        wn = self.board.wall_n_
        pawn = np.zeros((n, n), dtype=int)
        pawn[self.board.bot_pos_.y, self.board.bot_pos_.x] = 1
        pawn[self.board.player_pos_.y, self.board.player_pos_.x] = 1
        wall = np.zeros((wn, wn), dtype=int)
        for w in self.board.walls:
            wx, wy = w.pos
            wall[wy, wx] = 1 if w.orientation == Orientation.VER else 2
        return pawn, wall

    def _sync_snapshots_to_board(self):
        """After a programmatic apply (player UI / bot compute), advance the
        last-applied snapshot to match the new board state so perception
        catch-up frames don't re-trigger the same move."""
        pawn, wall = self._synthesize_grids_from_board()
        with self.perception_lock:
            self.prev_applied_pawn_grid = (
                self.last_applied_pawn_grid.copy()
                if self.last_applied_pawn_grid is not None else None
            )
            self.prev_applied_wall_grid = (
                self.last_applied_wall_grid.copy()
                if self.last_applied_wall_grid is not None else None
            )
            self.last_applied_pawn_grid = pawn
            self.last_applied_wall_grid = wall

    def on_board_update(self, msg: Int32MultiArray):
        if self.input_mode != "perception":
            return
        n = self.board.n_
        data = list(msg.data)
        if len(data) != n * n:
            self.get_logger().warn(
                f'Pawn grid size mismatch: got {len(data)} entries, expected {n*n}')
            return
        grid = np.flipud(np.array(data, dtype=int).reshape(n, n))
        self.get_logger().info(f"Got PAWN update:\n{grid}")
        with self.perception_lock:
            self.latest_pawn_grid = grid
            self._try_apply_perception_diff_locked()

    def on_wall_update(self, msg: Int32MultiArray):
        if self.input_mode != "perception":
            return
        wn = self.board.wall_n_
        data = list(msg.data)
        if len(data) != wn * wn:
            self.get_logger().warn(
                f'Wall grid size mismatch: got {len(data)} entries, expected {wn*wn}')
            return
        grid = np.flipud(np.array(data, dtype=int).reshape(wn, wn))
        self.get_logger().info(f"Got WALL update:\n{grid}")
        with self.perception_lock:
            self.latest_wall_grid = grid
            self._try_apply_perception_diff_locked()

    def _try_apply_perception_diff_locked(self):
        """Single entry point for both perception topics. Caller must hold
        perception_lock. Looks for exactly one pawn move OR one wall placement
        vs. the last-applied snapshot and applies it."""
        if self.bot_executing:
            return  # physical motion in progress; frames are unreliable
        if self.latest_pawn_grid is None or self.latest_wall_grid is None:
            return  # wait until we've seen at least one of each

        # First time both are available: baseline and return.
        if self.last_applied_pawn_grid is None or self.last_applied_wall_grid is None:
            self.last_applied_pawn_grid = self.latest_pawn_grid.copy()
            self.last_applied_wall_grid = self.latest_wall_grid.copy()
            return

        pawn_changed = not np.array_equal(
            self.latest_pawn_grid, self.last_applied_pawn_grid)
        wall_changed = not np.array_equal(
            self.latest_wall_grid, self.last_applied_wall_grid)

        if not pawn_changed and not wall_changed:
            return

        # Perception-lag suppression: if the latest frame still matches the
        # snapshot from before the most recent programmatic move, the camera
        # hasn't caught up yet -- wait silently.
        if (pawn_changed and self.prev_applied_pawn_grid is not None
                and np.array_equal(self.latest_pawn_grid, self.prev_applied_pawn_grid)):
            return
        if (wall_changed and self.prev_applied_wall_grid is not None
                and np.array_equal(self.latest_wall_grid, self.prev_applied_wall_grid)):
            return

        if pawn_changed and wall_changed:
            self.get_logger().warn(
                'Both pawn and wall grids changed simultaneously -- ignoring until stable')
            return

        move, description = (self._diff_pawn_move() if pawn_changed
                             else self._diff_wall_move())
        if move is None:
            return  # diff handler already logged why

        who = self.board.current_turn
        if not self.board.apply_move(move):
            self.get_logger().warn(
                f'Perceived {who} move ({description}) is illegal -- ignoring')
            return

        # Commit snapshot only after a successful apply. Synthesize from the
        # new board state (not from latest_*) so prev/last bookkeeping stays
        # consistent with the programmatic-move paths.
        self.prev_applied_pawn_grid = self.last_applied_pawn_grid.copy()
        self.prev_applied_wall_grid = self.last_applied_wall_grid.copy()
        pawn, wall = self._synthesize_grids_from_board()
        self.last_applied_pawn_grid = pawn
        self.last_applied_wall_grid = wall

        self.get_logger().info(f'Perception: {who} {description}')
        self.publish_board_state()

        if self.board.game_status != "in_progress":
            self.get_logger().info(f'Game over -- {self.board.game_status}')
            return

        if who == "player":
            self.request_bot_move()

    def _diff_pawn_move(self):
        past = self.last_applied_pawn_grid
        current = self.latest_pawn_grid
        disappeared, appeared = [], []
        for r in range(past.shape[0]):
            for c in range(past.shape[1]):
                p, n = int(past[r][c]), int(current[r][c])
                if p == 1 and n == 0:
                    disappeared.append((r, c))
                elif p == 0 and n == 1:
                    appeared.append((r, c))
        if len(disappeared) != 1 or len(appeared) != 1:
            self.get_logger().warn(
                f'Expected exactly one pawn move, got {len(disappeared)} disappeared '
                f'and {len(appeared)} appeared -- ignoring')
            return None, None

        from_row, from_col = disappeared[0]
        to_row, to_col = appeared[0]
        from_pos = Pawn(from_col, from_row)
        to_pos = Pawn(to_col, to_row)

        if from_pos == self.board.player_pos_:
            who = "player"
        elif from_pos == self.board.bot_pos_:
            who = "bot"
        else:
            self.get_logger().warn(
                f'Moved pawn at ({from_pos.x},{from_pos.y}) does not match '
                f'player ({self.board.player_pos_.x},{self.board.player_pos_.y}) '
                f'or bot ({self.board.bot_pos_.x},{self.board.bot_pos_.y}) -- ignoring')
            return None, None

        if who != self.board.current_turn:
            self.get_logger().warn(
                f'Detected {who} pawn move but it is {self.board.current_turn}\'s turn -- ignoring')
            return None, None

        move = Move(move_type=MoveType.PAWN, target=to_pos)
        desc = f'pawn moved from ({from_pos.x},{from_pos.y}) to ({to_pos.x},{to_pos.y})'
        return move, desc

    def _diff_wall_move(self):
        past = self.last_applied_wall_grid
        current = self.latest_wall_grid
        appeared, removed = [], []
        for r in range(past.shape[0]):
            for c in range(past.shape[1]):
                p, n = int(past[r][c]), int(current[r][c])
                if p == 0 and n != 0:
                    appeared.append((c, r, n))
                elif p != 0 and n == 0:
                    removed.append((c, r, p))
        if removed:
            self.get_logger().warn(
                f'Wall(s) disappeared from perception: {removed} -- ignoring')
            return None, None
        if len(appeared) != 1:
            self.get_logger().warn(
                f'Expected exactly one new wall, got {len(appeared)} -- ignoring')
            return None, None

        wx, wy, val = appeared[0]
        if val == 1:
            orient = Orientation.VER
        elif val == 2:
            orient = Orientation.HOR
        else:
            self.get_logger().warn(f'Unknown wall value {val} -- ignoring')
            return None, None

        move = Move(move_type=MoveType.WALL,
                    wall=Wall(pos=(wx, wy), orientation=orient))
        desc = f'placed {orient.name} wall at ({wx},{wy})'
        return move, desc


def main(args=None):
    rclpy.init(args=args)
    node = StateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()