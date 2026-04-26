# quoridor_ai_move

Deep Q-Learning (CNN + Reinforcement Learning) replacement for
`quoridor_move_decision`. Drop-in compatible: same topics, same JSON schema.

## Architecture

- **State encoding** (`encoder.py`): 5×5×7 tensor — my pawn, opponent pawn,
  horizontal walls, vertical walls, my walls remaining, opponent walls
  remaining, turn indicator. Always rendered from the perspective of the
  side-to-move so the network sees the game symmetrically.
- **Action space** (56 total, for n=5): 24 relative pawn offsets plus
  4×4×2 wall placements.
- **Model** (`model.py`): convolutional residual tower → dense head →
  Q-values over all 56 actions. Illegal actions are masked out before
  argmax / before the Bellman target.
- **Agent** (`agent.py`): standard DQN — online network, target network,
  epsilon-greedy with legal-action mask, Huber-free MSE loss, Adam.
- **Replay buffer** (`replay_buffer.py`): numpy-backed ring buffer.
- **Self-play** (`self_play.py`): generates per-side MDP transitions;
  intermediate rewards are 0, terminal reward ±1.

## Training from scratch

Single machine (uses GPU automatically if available):

```bash
ros2 run quoridor_ai_move train --episodes 5000 \
    --model-dir ~/quoridor_models/v1
```

Resume training:

```bash
ros2 run quoridor_ai_move train --resume --model-dir ~/quoridor_models/v1
```

## Distributed training across multiple machines

TensorFlow's `MultiWorkerMirroredStrategy` is used. Put the model dir on
shared storage (NFS, etc.) and set `TF_CONFIG` on each worker:

```bash
# Worker 0
export TF_CONFIG='{"cluster":{"worker":["host0:12345","host1:12345"]},"task":{"type":"worker","index":0}}'
ros2 run quoridor_ai_move train --distributed --episodes 10000 \
    --model-dir /mnt/shared/quoridor_models/v1
```

```bash
# Worker 1 (same command, different index)
export TF_CONFIG='{"cluster":{"worker":["host0:12345","host1:12345"]},"task":{"type":"worker","index":1}}'
ros2 run quoridor_ai_move train --distributed --episodes 10000 \
    --model-dir /mnt/shared/quoridor_models/v1
```

Both workers self-play independently; gradients are all-reduced via NCCL
(on GPU) or gRPC (on CPU). The chief (task index 0) saves checkpoints.

## Training a new model against an existing one

Once you have a `v1`, train a `v2` that surpasses it:

```bash
ros2 run quoridor_ai_move train_vs_model \
    --teacher   ~/quoridor_models/v1 \
    --model-dir ~/quoridor_models/v2 \
    --episodes  5000
```

Options:
- `--student-side {bot,player,both}` — which side the student plays
- `--swap-each-episode` — alternate sides each game (useful with a fixed side)
- `--teacher-eps 0.05` — small exploration noise for the teacher
- `--distributed` — same multi-worker mechanism

## Deploying as the game engine

```bash
ros2 launch quoridor_ai_move ai_move.launch.py \
    model_dir:=$HOME/quoridor_models/v2 \
    online_learning:=true \
    save_after_game:=true
```

Launch parameters:
- `model_dir` — where to load/save weights
- `online_learning` — record real-game transitions and train after each game
- `save_after_game` — overwrite `model_dir` after every finished game
- `epsilon` — inference exploration (default 0, i.e. greedy)
- `side` — which side the node plays (`bot` or `player`)

At runtime the node listens on `/quoridor/compute_move_request` and
publishes moves on `/quoridor/compute_move_response`, exactly like the
original minimax engine. When the state manager publishes a terminal
board, the node closes the trajectory, runs a short online update, and
saves new weights.

## GPU requirements

Any CUDA-capable GPU with a working `tensorflow` install (TF ≥ 2.12
recommended). Memory growth is enabled automatically. The 5×5 board model
is small enough to train on a single consumer GPU; multi-GPU and multi-
worker modes are there for faster iteration, not necessity.
