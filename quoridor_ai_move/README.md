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
| `agent.py` | DQN — online + target network, soft target updates (`tau`), Huber loss, gradient clipping, masked argmax. Optimizer is built eagerly inside `strategy.scope()` for distributed compatibility. |
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

Both terminal and shaped rewards are used by default:

- Terminal: `+1` on win, `-1` on loss, `-0.1` on max-plies timeout, `-1` on illegal action.
- Per-step (PBRS): `γ · Φ(s') − Φ(s)` where `Φ` is the side-relative
  potential `coef · (opp_dist − my_dist + 0.1 · (my_walls − opp_walls))`.
  `Φ(terminal) = 0`, so terminal rewards stay intact.

PBRS is policy-invariant (Ng-Harada-Russell): the optimal policy under
shaped rewards is the same as under sparse ±1, only learning speed
changes. Disable with `shaping_coef=0`.

---

## Launch files

| File | Purpose |
|------|---------|
| `train_ros.launch.py` | Full ROS training stack (state_manager + move_decision + web UI + trainer) |
| `play_vs_ai.launch.py` | Play against the trained AI in the web UI |
| `ai_move.launch.py` | Inference node only (drop-in for move_decision) |

---

## Training pipelines

### 1. Against the minimax engine — recommended starting point

Pure Python, no ROS. Fastest path to a competent agent.

```bash
python -m quoridor_ai_move.train_vs_minimax \
    --model-dir ./quoridor_models/v_mm \
    --tb-log-dir ./quoridor_tensorboard \
    --episodes    5000 \
    --minimax-depth 2 \
    --shaping-coef  0.3 \
    --swap-each-episode
```

To resume after stopping:

```bash
python -m quoridor_ai_move.train_vs_minimax \
    --model-dir ./quoridor_models/v_mm \
    --resume \
    --minimax-depth 2 \
    --episodes    5000 \
    --shaping-coef  0.3 \
    --eps-start 0.05 \
    --eps-end   0.05 \
    --swap-each-episode
```

`--eps-start 0.05 --eps-end 0.05` keeps epsilon fixed so a resumed model
does not re-explore from scratch.

Useful flags:
- `--minimax-depth N` — teacher search depth. **Start at 2.** Only bump to
  3 once `episode/win_rate_50ep` is consistently above 60%. Jumping too
  early causes catastrophic win-rate collapse (the depth-3 minimax on a
  5×5 board wins in the minimum 4 moves).
- `--shaping-coef C` — PBRS strength. `0.3` recommended. `0` disables.
- `--student-side {bot,player}` — which side the student plays.
- `--swap-each-episode` — alternate sides each game.
- `--resume` — load existing weights from `--model-dir` first.
- `--tau` — soft target update rate (default `0.005`; `1.0` = hard copy).

### 2. ROS-driven training with web visualisation

Plays through `state_manager` against the minimax bot. Every move flows
through the normal topic graph so the web UI renders training games live.

```bash
ros2 launch quoridor_ai_move train_ros.launch.py \
    model_dir:=$HOME/quoridor_models/v_ros \
    shaping_coef:=0.1
```

To resume:

```bash
ros2 launch quoridor_ai_move train_ros.launch.py \
    model_dir:=$HOME/quoridor_models/v_ros \
    shaping_coef:=0.1 \
    resume:=true
```

Or run manually (set to manual mode in the web UI first):

```bash
ros2 run quoridor_ai_move train_ros --ros-args \
    -p model_dir:=$HOME/quoridor_models/v_ros \
    -p shaping_coef:=0.1 \
    -p resume:=true
```

**Note:** All parameters must use `--ros-args -p param:=value` syntax.
Bare `--flag` arguments are silently ignored by ROS nodes.

ROS parameters:
- `model_dir`, `tb_log_dir` — checkpoint and TensorBoard paths.
- `shaping_coef` — PBRS strength (default `0.1`).
- `resume` — load existing weights on start (default `false`).
- `updates_per_game` — gradient steps per finished game (default `16`).
- `target_sync_every` — grad-steps between target net syncs.
- `save_every_episodes` — checkpoint cadence.
- `auto_restart` / `max_episodes` — loop control.
- Plus standard hyperparameters: `lr`, `gamma`, `tau`, `batch_size`,
  `replay_capacity`, `eps_*`, `filters`, `blocks`.

### 3. Pure self-play

Both sides controlled by the same (improving) network.

```bash
python -m quoridor_ai_move.train \
    --model-dir ./quoridor_models/v_self \
    --tb-log-dir ./quoridor_tensorboard \
    --episodes 5000
```

Best used **after** an initial pass against minimax: once the network
wins consistently, self-play produces stronger gradients than losing every
game to a fixed teacher. GPU stays fully utilised since every move is a
network forward pass (no CPU minimax overhead).

### 4. Student vs frozen DQN teacher (iterative leagues)

```bash
python -m quoridor_ai_move.train_vs_model \
    --teacher   ./quoridor_models/v1 \
    --model-dir ./quoridor_models/v2 \
    --episodes  5000
```

Use once you have a strong `v1` and want strict generation-over-generation
improvement without the CPU overhead of minimax.

### TensorBoard

All training pipelines log to TensorBoard:

- `episode/reward`, `episode/length`, `episode/win`, `episode/win_rate_50ep`
- `episode/loss`, `episode/loss_50ep`, `episode/mean_q` (ROS trainer only)
- `training/epsilon`, `training/buffer_size`, `training/grad_steps`

```bash
tensorboard --logdir=./quoridor_tensorboard
```

---

## Playing against the AI

Once you have a trained model, launch the full play stack:

```bash
ros2 launch quoridor_ai_move play_vs_ai.launch.py \
    model_dir:=$HOME/quoridor_models/v_mm
```

Open `http://localhost:8088`. You play as player, the AI plays as bot.

To make the AI keep learning from your games:

```bash
ros2 launch quoridor_ai_move play_vs_ai.launch.py \
    model_dir:=$HOME/quoridor_models/v_mm \
    online_learning:=true \
    save_after_game:=true
```

Parameters:
- `model_dir` — load/save weights here.
- `epsilon` — exploration rate (default `0.0` = fully greedy).
- `side` — which side the AI plays (`bot` or `player`).
- `online_learning` — accumulate transitions and update after each game.
- `save_after_game` — overwrite checkpoint after every finished game.

Or run just the inference node as a drop-in for `quoridor_move_decision`:

```bash
ros2 launch quoridor_ai_move ai_move.launch.py \
    model_dir:=$HOME/quoridor_models/v_mm \
    epsilon:=0.0 \
    side:=bot
```

### What is epsilon?

Epsilon controls how often the AI plays a random legal move instead of its
best move. `0.0` = always plays best (use for playing against it). `1.0` =
fully random. During training it starts at `1.0` and decays to `0.05`.

---

## Distributed training across multiple machines

`train.py`, `train_vs_minimax.py`, and `train_vs_model.py` accept
`--distributed`, which uses `tf.distribute.MultiWorkerMirroredStrategy`.
Multi-GPU on a single host is auto-detected (`MirroredStrategy`).

### Requirements

1. **Shared storage** — `--model-dir` and `--tb-log-dir` on a path both
   machines can write (NFS, SSHFS). Only the chief (worker 0) writes
   checkpoints.
2. **Identical software** — same TensorFlow version, same Python version,
   same `colcon build` of `quoridor_ai_move`.
3. **Matching device types** — all workers must use the same device type
   (all GPU or all CPU). If one machine has a GPU and the other does not,
   hide the GPU on the machine that has one:
   ```bash
   export CUDA_VISIBLE_DEVICES=""
   ```
   Verify with:
   ```bash
   python -c "import tensorflow as tf; print(tf.config.list_physical_devices('GPU'))"
   ```
   Both machines should print the same result (`[]` for CPU-only).

4. **GPU libraries (WSL)** — if using WSL Ubuntu, TF cannot find CUDA
   libraries from pip packages without this:
   ```bash
   export LD_LIBRARY_PATH=$(python -c "import os,nvidia;base=os.path.dirname(nvidia.__file__);print(':'.join([os.path.join(base,d,'lib') for d in os.listdir(base) if os.path.isdir(os.path.join(base,d,'lib'))]))" 2>/dev/null):$LD_LIBRARY_PATH
   ```
   Add to `~/.bashrc` to make permanent.

5. **Set `TF_CONFIG`** on each machine — the `cluster.worker` list must be
   **byte-identical** on both machines (same IPs, same order, same port).
   Only `task.index` differs:

```bash
# Worker 0 (chief, e.g. 192.168.6.122)
export TF_CONFIG='{"cluster":{"worker":["192.168.6.122:12345","192.168.6.39:12345"]},"task":{"type":"worker","index":0}}'

# Worker 1 (e.g. 192.168.6.39)
export TF_CONFIG='{"cluster":{"worker":["192.168.6.122:12345","192.168.6.39:12345"]},"task":{"type":"worker","index":1}}'
```

**Common mistake:** putting both IPs in one string instead of two:
```bash
# WRONG - both IPs in one string, chief rejects worker 1 as "unexpected"
"worker": ["192.168.6.122:12345, 192.168.6.39:12345"]

# CORRECT - two separate strings
"worker": ["192.168.6.122:12345", "192.168.6.39:12345"]
```

Verify with: `echo "$TF_CONFIG" | python -m json.tool`

6. **Launch the same command on every machine** (chief first):

```bash
python -m quoridor_ai_move.train_vs_minimax \
    --distributed \
    --model-dir   /mnt/shared/quoridor_models/v_mm \
    --tb-log-dir  /mnt/shared/quoridor_tensorboard \
    --episodes    10000 \
    --minimax-depth 2 \
    --shaping-coef  0.3 \
    --swap-each-episode
```

### Sanity check (one machine, 2 workers)

```bash
# Terminal 1
export TF_CONFIG='{"cluster":{"worker":["localhost:12345","localhost:12346"]},"task":{"type":"worker","index":0}}'
python -m quoridor_ai_move.train_vs_minimax --distributed --model-dir /tmp/mm --episodes 50

# Terminal 2
export TF_CONFIG='{"cluster":{"worker":["localhost:12345","localhost:12346"]},"task":{"type":"worker","index":1}}'
python -m quoridor_ai_move.train_vs_minimax --distributed --model-dir /tmp/mm --episodes 50
```

---

## Recommended training schedule

A typical path from random to competent:

1. **Bootstrap** — `train_vs_minimax --minimax-depth 2 --shaping-coef 0.3`
   for ~3000 episodes. Watch `episode/win_rate_50ep` in TensorBoard.
2. **Consolidate at depth 2** — keep training until win rate is
   consistently **above 60%**. Do not advance early — jumping to depth 3
   when win rate is below 50% causes the agent to collapse back to 0%
   because depth-3 minimax wins in the minimum 4 moves.
3. **Step to depth 3** — `--minimax-depth 3 --resume --eps-start 0.05 --eps-end 0.05`
   for another 3-5k episodes with `--swap-each-episode`.
4. **Self-play** — once win rate vs depth-3 exceeds ~50%, switch to
   `train.py` (self-play) for stronger gradients and full GPU utilisation.
5. **(Optional) League** — train `v2` against frozen `v1` with
   `train_vs_model` for strict generation-over-generation improvement.
6. **Deploy** — `play_vs_ai.launch.py` with `online_learning:=true`.

Watch `episode/win_rate_50ep` and `episode/mean_q` in TensorBoard to
decide when to advance. Loss curves alone won't tell you if the agent
is actually getting better.

---

## Troubleshooting

### Win rate dropped after resuming at higher depth

**Symptom:** `episode/win_rate_50ep` collapses, `episode/length` drops to
4-5 (minimum possible game length on 5×5).

**Cause:** The minimax depth increase is too large. Depth-3 minimax plays
near-optimally and wins in the minimum number of moves before the agent
can learn anything.

**Fix:** Resume at the previous depth and set epsilon to its final value
so the model does not re-explore:
```bash
python -m quoridor_ai_move.train_vs_minimax \
    --model-dir ./quoridor_models/v_mm --resume \
    --minimax-depth 2 --eps-start 0.05 --eps-end 0.05 \
    --shaping-coef 0.3 --swap-each-episode
```

### Distributed: "Unexpected task registered with task_name=.../task:1"

The chief's `TF_CONFIG` cluster list is wrong. Check that both IPs are
separate strings and that the port is open on the chief machine.

### Distributed: "incompatible device type CPU / GPU"

Workers have different hardware. Use `CUDA_VISIBLE_DEVICES=""` on the
machine with the GPU to force CPU-only on both sides.

### train_ros hangs on first step

State manager must be set to **manual mode** in the web UI before the
trainer sends its first `start` command. If it still hangs, verify all
nodes are running:
```bash
ros2 node list
ros2 topic echo /quoridor/board_state
```

### ROS node ignoring parameters

Use `--ros-args -p param:=value` syntax. Bare `--flag` arguments are
silently ignored by ROS2 nodes:
```bash
# WRONG
ros2 run quoridor_ai_move train_ros --resume --model-dir /path

# CORRECT
ros2 run quoridor_ai_move train_ros --ros-args -p resume:=true -p model_dir:=/path
```

---

## GPU requirements

Any CUDA-capable GPU with a working `tensorflow[and-cuda]` install
(TF >= 2.14 recommended). Memory growth is enabled automatically. The
5×5 board model is small enough to train on a single consumer GPU.

The main training bottleneck is the **CPU minimax search**, not the GPU
— the GPU sits idle between games. To maximise GPU utilisation, use
self-play (`train.py`) where both moves are network forward passes, or
increase batch size (`--batch-size 512`) to do more gradient steps per
game.
