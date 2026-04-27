# quoridor_ai_move

Deep Q-Learning (CNN + Reinforcement Learning) move engine for Quoridor.
Drop-in replacement for the minimax engine in `quoridor_move_decision`:
same topics, same JSON schema. Uses the same `QuoridorBoard` from
`quoridor_game.quoridor_utils` so the rest of the stack is untouched.

## Architecture

| File | Role |
|------|------|
| `encoder.py` | Board → `(5, 5, 7)` tensor; flat 56-action index; legal-action mask. Always rendered from the side-to-move's POV (board is y-flipped when playing as `player`). |
| `model.py` | Residual CNN: 3×3 conv stem → 4 residual blocks (64 filters) → dense 256 → 56-way Q-head. L2 reg + dropout. |
| `agent.py` | DQN — online + target network, soft target updates (`tau`), Huber loss, gradient clipping, masked argmax. |
| `replay_buffer.py` | Numpy-backed circular buffer of `(s, a, r, s', done, next_legal_mask)`. |
| `self_play.py` | Plays one game between two policies, returns per-side transitions with optional PBRS shaping. |
| `reward.py` | Potential-based reward shaping using path-difference + wall-difference (same features as minimax `_evaluate`). |
| `minimax_policy.py` | Pure-Python alpha-beta search (mirror of `quoridor_game.move_decision`) wrapped as a `Policy` for `play_game`. |
| `train.py` | Self-play training loop with TensorBoard logging. |
| `train_vs_model.py` | Train a fresh student against a frozen DQN teacher checkpoint. |
| `train_vs_minimax.py` | Train against the alpha-beta minimax engine (no ROS, fastest). |
| `train_ros.py` | Train through `state_manager` + `quoridor_move_decision` so games show in the web UI. |
| `ai_move_node.py` | Inference node: drop-in replacement for the minimax engine, with optional online learning. |

### State (7 channels, `(5, 5, 7)`)
0. my pawn (one-hot)
1. opponent pawn (one-hot)
2. horizontal walls (one-hot on 4×4 sub-block)
3. vertical walls
4. my walls remaining (broadcast plane, normalized)
5. opp walls remaining (broadcast plane, normalized)
6. side-to-move flag (broadcast plane)

### Action space (56)
- `0..23` — pawn moves: `(dx, dy) ∈ [-2, 2]² \ {(0, 0)}` in agent frame.
- `24..39` — horizontal wall placements (4×4 grid).
- `40..55` — vertical wall placements (4×4 grid).

Illegal actions are masked to −∞ before argmax in `select_action` and
masked in the Bellman target's `max Q(s')`.

### Reward

By default both terminal and shaped rewards are used:

- Terminal: `+1` on win, `-1` on loss, `-0.1` on max-plies timeout, `-1` on illegal action.
- Per-step (PBRS): `γ · Φ(s') − Φ(s)` where `Φ` is the side-relative
  potential `coef · (opp_dist − my_dist + 0.1 · (my_walls − opp_walls))`.
  `Φ(terminal) = 0`, so terminal rewards stay intact.

PBRS is policy-invariant (Ng-Harada-Russell): the optimal policy under
shaped rewards is the same as under sparse ±1, only learning speed
changes. Disable with `shaping_coef=0`.

## Training pipelines

### 1. Against the minimax engine (recommended starting point)

Pure Python, no ROS. Fastest path to a competent agent.

```bash
ros2 run quoridor_ai_move train_vs_minimax \
    --model-dir   ./quoridor_models/v_mm \
    --tb-log-dir  ./quoridor_tensorboard \
    --episodes    5000 \
    --minimax-depth 3 \
    --shaping-coef  0.1 \
    --swap-each-episode
```

Useful flags:
- `--minimax-depth N` — teacher search depth. Drop to 2 for warm-up if
  the agent can't get any wins; bump back to 3 once it's at ~50%.
- `--shaping-coef C` — PBRS strength. `0.1` default. `0` disables.
- `--student-side {bot,player}` — which side the student plays.
- `--swap-each-episode` — alternate sides each game.
- `--resume` — load existing student weights from `--model-dir` first.
- `--tau` — soft target update rate (default `0.005`; `1.0` = hard copy).

### 2. Pure self-play

Both sides controlled by the same (improving) network.

```bash
ros2 run quoridor_ai_move train --episodes 5000 \
    --model-dir ./quoridor_models/v_self \
    --tb-log-dir ./quoridor_tensorboard
```

Best used **after** an initial pass against minimax: once the network
isn't completely random, self-play produces stronger gradients than
losing every game to a fixed teacher.

### 3. Student vs frozen DQN teacher (iterative leagues)

```bash
ros2 run quoridor_ai_move train_vs_model \
    --teacher   ./quoridor_models/v1 \
    --model-dir ./quoridor_models/v2 \
    --episodes  5000
```

Use this once you have a strong `v1` and want to enforce strict
generation-over-generation improvement.

### 4. ROS-driven training with web visualization

Plays as the **player** through the real `state_manager`, against the
existing `quoridor_move_decision` minimax engine. Every move flows
through the normal topic graph so the web UI renders training games
live.

Run each in its own terminal:

```bash
# Terminal 1 - game state owner. Open the web UI and toggle to "manual" mode.
ros2 run quoridor_game state_manager

# Terminal 2 - minimax bot
ros2 run quoridor_move_decision move_decision

# Terminal 3 - web UI (http://localhost:8088)
ros2 run quoridor_game web_interface

# Terminal 4 - the DQN trainer
ros2 run quoridor_ai_move train_ros --ros-args \
    -p model_dir:=$HOME/quoridor_models/v_ros \
    -p tb_log_dir:=./quoridor_tensorboard \
    -p shaping_coef:=0.1

# Terminal 5 - metrics
tensorboard --logdir=./quoridor_tensorboard
```

The trainer drives the game lifecycle itself: it sends `start` on
`/quoridor/game_command` for each new episode, plays as `player` via
`/quoridor/player_move`, watches `/quoridor/board_state` for terminal
states, then trains and restarts.

ROS parameters:
- `model_dir`, `tb_log_dir` — checkpoint and TensorBoard paths.
- `shaping_coef` — PBRS strength (default `0.1`).
- `updates_per_game` — gradient steps per finished game (default `16`).
- `target_sync_every` — grad-steps between target net syncs.
- `save_every_episodes` — checkpoint cadence.
- `auto_restart` / `max_episodes` — loop control.
- Plus the standard hyperparameters: `lr`, `gamma`, `tau`, `batch_size`,
  `replay_capacity`, `eps_*`, `filters`, `blocks`, `resume`.

### TensorBoard scalars

All training pipelines log:

- `episode/reward`, `episode/length`, `episode/win`, `episode/win_rate_50ep`
- `episode/loss`, `episode/loss_50ep`, `episode/mean_q` (where applicable)
- `training/epsilon`, `training/buffer_size`, `training/grad_steps`

```bash
tensorboard --logdir=./quoridor_tensorboard
```

## Distributed training across multiple machines

`train.py`, `train_vs_minimax.py`, and `train_vs_model.py` accept
`--distributed`, which uses `tf.distribute.MultiWorkerMirroredStrategy`.
Multi-GPU on a single host is auto-detected (`MirroredStrategy`).

### Setup

1. **Shared storage** — `--model-dir` and `--tb-log-dir` on a path both
   machines can write to (NFS, SSHFS, network share). Only the chief
   (worker 0) writes checkpoints; other workers no-op `save`.
2. **Identical software** — same TensorFlow version, same Python
   version, same `colcon build` of `quoridor_ai_move`. Pick a port
   open between the machines (e.g. `12345`).
3. **Set `TF_CONFIG`** on each machine — same `cluster` list everywhere,
   only the `index` changes:

```bash
# Worker 0 (chief, host 10.0.0.1)
export TF_CONFIG='{
  "cluster": {"worker": ["10.0.0.1:12345", "10.0.0.2:12345"]},
  "task":    {"type": "worker", "index": 0}
}'

# Worker 1 (host 10.0.0.2)
export TF_CONFIG='{
  "cluster": {"worker": ["10.0.0.1:12345", "10.0.0.2:12345"]},
  "task":    {"type": "worker", "index": 1}
}'
```

4. **Launch the same command on every machine**:

```bash
ros2 run quoridor_ai_move train_vs_minimax \
    --distributed \
    --model-dir   /mnt/shared/quoridor_models/v_mm \
    --tb-log-dir  /mnt/shared/quoridor_tensorboard \
    --episodes    10000 \
    --minimax-depth 3 \
    --shaping-coef  0.1 \
    --swap-each-episode
```

The processes rendezvous over gRPC, build the model under the
strategy scope, and start training. Each worker plays its own games
(CPU minimax parallelizes linearly with workers — that's the actual
speed-up); gradients are all-reduced (NCCL on GPU, gRPC on CPU) every
training step so weights stay in sync.

### What you get
- ~`N×` games per wall-clock hour with `N` workers.
- Synchronous gradient updates — identical weights across workers.
- One TensorBoard run — chief writes; no merging needed.

### Caveats
- **Firewalls** — open the rendezvous port between hosts.
  `nc -lv 12345` / `nc 10.0.0.1 12345` is the quickest test.
- **Hostnames vs IPs** — use IPs unless DNS is rock solid; mismatched
  name resolution causes opaque rendezvous timeouts.
- **Replay buffers are per-worker** (independent). Warmup
  (`buf >= batch_size`) happens locally on each worker.
- **Don't run two workers on one GPU** unless you cap memory.
- **Chief failure** stops the job. Relaunch with `--resume`.

### Quick sanity check (one machine, 2 workers)

```bash
# Terminal 1
export TF_CONFIG='{"cluster":{"worker":["localhost:12345","localhost:12346"]},"task":{"type":"worker","index":0}}'
ros2 run quoridor_ai_move train_vs_minimax --distributed --model-dir /tmp/mm --episodes 50

# Terminal 2
export TF_CONFIG='{"cluster":{"worker":["localhost:12345","localhost:12346"]},"task":{"type":"worker","index":1}}'
ros2 run quoridor_ai_move train_vs_minimax --distributed --model-dir /tmp/mm --episodes 50
```

If the loss curves match a single-process run, multi-host will work too.

## Deploying as the game engine

Once you have a trained model, run the inference node in place of
`quoridor_move_decision/move_decision`:

```bash
ros2 launch quoridor_ai_move ai_move.launch.py \
    model_dir:=$HOME/quoridor_models/v_mm \
    online_learning:=true \
    save_after_game:=true \
    epsilon:=0.0 \
    side:=bot
```

Parameters:
- `model_dir` — load/save weights here.
- `online_learning` — keep accumulating transitions during real games
  and run a short update at the end of each.
- `save_after_game` — overwrite `model_dir` after every finished game.
- `epsilon` — inference exploration (default `0.0`, i.e. greedy).
- `side` — which side this node plays (`bot` or `player`).

The node listens on `/quoridor/compute_move_request` and publishes on
`/quoridor/compute_move_response`, exactly like the original engine.
When `state_manager` publishes a terminal board, it closes the
trajectory, runs a short online update, and saves new weights.

## Recommended training schedule

A typical path from random to competent:

1. **Bootstrap** — `train_vs_minimax --minimax-depth 2 --shaping-coef 0.1`
   for ~2000 episodes. The agent needs to start *winning* to learn anything.
2. **Strengthen** — bump to `--minimax-depth 3`, run another 3-5k episodes
   with `--swap-each-episode --resume`.
3. **Self-play** — once `episode/win_rate_50ep` against depth-3 is over
   ~50%, switch to `train` (self-play, `--resume`) for stronger gradients.
4. **(Optional) League** — train `v2` against frozen `v1` with
   `train_vs_model` to enforce strict generation-over-generation gains.
5. **Deploy** — `ai_move.launch.py` with `online_learning:=true`. The
   model keeps improving from real games.

Watch `episode/win_rate_50ep` and `episode/mean_q` in TensorBoard to
decide when to advance. Loss curves alone won't tell you if the agent
is actually getting better.

## GPU requirements

Any CUDA-capable GPU with a working `tensorflow` install (TF ≥ 2.12
recommended). Memory growth is enabled automatically. The 5×5 board
model is small enough to train on a single consumer GPU; multi-GPU and
multi-worker modes exist for faster iteration, not necessity.
