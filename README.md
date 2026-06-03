# QuorZero

**An AlphaZero-style AI for 5×5 Quoridor**

A self-play reinforcement learning engine that learns Quoridor from scratch using a dual-head neural network and Monte Carlo Tree Search, deployed as the brain of a UR3e robotic arm that plays against a human on a physical board.

[![Code](https://img.shields.io/badge/GitHub-quoridor__alphazero-black?logo=github)](https://github.com/BinethAtukorala/RS2-Quoridor/tree/integration/quoridor_alphazero)
[![Full Repo](https://img.shields.io/badge/GitHub-full%20repository-black?logo=github)](https://github.com/BinethAtukorala/RS2-Quoridor)

| | |
|---|---|
| Action space | 56 moves |
| Board | 5×5 |
| Network parameters | ~80k |
| Games trained | ~125k |

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Team](#2-team)
3. [Approach](#3-approach)
   - [Why AlphaZero, not DQN?](#31-why-alphazero-not-dqn)
   - [The Self-Play Loop](#32-the-self-play-loop)
   - [Training Objective](#33-training-objective)
   - [Network Architecture](#34-network-architecture)
4. [Results & Training Signals](#4-results--training-signals)
5. [Benchmark Results](#5-benchmark-results)
6. [QuorZero: Installation & Usage](#6-quorzero-installation--usage)
7. [Robotic System](#7-robotic-system)
   - [Perception](#71-perception)
   - [Robot Motion Control](#72-robot-motion-control)
   - [Full System Launch](#73-full-system-launch)
8. [Hardware Requirements](#8-hardware-requirements)
9. [Troubleshooting](#9-troubleshooting)

---

## 1. Project Overview

**QuorZero** is the AI component of a UR3e robotic arm system that plays *Quoridor* — a two-player strategy board game — against a human opponent. This document focuses on the `quoridor_alphazero` ROS 2 package: the neural network, the search algorithm, and the training pipeline.

Quoridor is a perfect-information two-player game with a branching factor that rewards strong planning. We replaced an earlier Deep Q-Network (DQN) baseline with an AlphaZero-style approach: a single network with a *policy head* and a *value head*, trained on data produced by Monte Carlo Tree Search that uses the same network as the leaf evaluator. The network guides a search that is itself guided by the network. Unlike DQN, this configuration handles the two-player setting natively.

---

## 2. Team

| Member | Role |
|---|---|
| **Bineth Atukorala** | AI / AlphaZero (QuorZero) — designed and implemented the AlphaZero pipeline: dual-head network, MCTS, self-play loop, and the ROS 2 inference node. |
| **Bihan Sudusinghe** | DQN model — built the initial DQN baseline (same encoder and action space, different learning algorithm); improved QuorZero training performance by over 200% by fixing multiprocessing bugs. Also built the perception subsystem. |
| **Binada Sudusinghe** | Benchmarking & evaluation — designed the head-to-head benchmark system comparing QuorZero against DQN and minimax, and analysed win-rate / game-length statistics. Also built the robot motion control subsystem. |

---

## 3. Approach

### 3.1. Why AlphaZero, not DQN?

DQN was designed for single-agent environments. In a two-player game, the Bellman bootstrap `r + γ·max Q(s', a)` confuses "my move quality" with "my opponent's reply quality" because `s'` is the state *after* the opponent moves. This produces recurring issues in self-play: mutual stalling, oscillating policies, and overfitted learning.

AlphaZero handles two players natively:

- The **value head** is trained against the actual game outcome `z ∈ {−1, 0, +1}`.
- The **policy head** is trained against the MCTS visit-count distribution — a strictly stronger signal than the network's own best guess.
- **Dual-side learning** in MCTS flips the sign of value at every level. What's good for the child is bad for the parent, so the network learns how to attack *and* defend.

### 3.2. The Self-Play Loop

Each self-play game uses MCTS for *both* sides with the PUCT (Predictor + Upper-Confidence Tree) variant. After every move the visit-count distribution π is recorded alongside the state. When the game ends, the outcome `z` is stamped back onto every sample and the triples `(s, π, z)` are pushed into a circular replay buffer. A small number of Adam optimiser steps follow each game.

```
while game not over:
    counts = mcts.run(board)
    π = visit_counts_to_policy(counts, τ)
    record (state, π, side)
    sample action ~ π
    board.apply_move(...)

z = ±1 / 0 from each side's point of view
push every (state, π, z) into ReplayBuffer
```

### 3.3. Training Objective

The policy head is trained to imitate MCTS's visit-count distribution over moves; the value head is trained to predict the eventual game outcome `z ∈ {-1, 0, +1}`. Because the same network is the evaluator inside MCTS, every training step makes the search sharper, which produces better data for the next step — the network continuously improves.

```
loss = − Σ π_mcts · log softmax(p_logits)   # policy cross-entropy
       + (z − tanh(v_logit))²                # value mean-squared error
       + L2(weights)                         # L2 weight decay
```

### 3.4. Network Architecture

The network uses a convolutional stem followed by 3 residual blocks (~80k parameters total), with two output heads:

- **Policy head** — 56 logits covering all legal pawn moves and wall placements on the 5×5 board.
- **Value head** — a scalar outcome prediction.

The self-reinforcing training loop flows: **Network → Self-play + MCTS → Replay Buffer → Train Step (Adam) → Network**.

---

## 4. Results & Training Signals

Headline metrics tracked in TensorBoard during training:

| Metric | Range | Meaning |
|---|---|---|
| `loss/policy` | 4.0 → 2.5 | Network agreeing with MCTS. Stuck at `ln 56 ≈ 4` means uniform predictions — search isn't biting yet. |
| `loss/value` | 0.5 → 0.1 | Network learning to predict outcomes. A rising value loss flags overfitting or a stale buffer. |
| `game/avg_length_50` | 15–35 plies | Indicates decisive games. Pinned at `--max-plies` means search is too weak to find winning lines. |
| `game/decisive_rate_50` | → 1.0 | Fraction of recent games finishing cleanly. Approaches 1 as the network learns to close out positions. |

---

## 5. Benchmark Results

QuorZero played 50 games against each of five baselines — 25 as first player and 25 as second — for 250 head-to-head matches total.

| | |
|---|---|
| **76.4%** overall win rate (191 / 250) | **150 / 150** vs every DQN variant |
| **~820 ms** avg time per move (400 simulations) | **0** draws produced |

**Per-opponent breakdown:**

| Opponent | Result | Notes |
|---|---|---|
| Minimax-d2 | 27 / 50 · **54%** | Edges out the shallow baseline. 14/25 as first player, 13/25 as second — effectively neutral on side. |
| Minimax-d3 | 14 / 50 · **28%** | Deep minimax remains stronger overall, but QuorZero wins 14/25 when it moves first, matching the deeper search when it has the initiative. |
| DQN_4 | 50 / 50 · **100%** | Fully defeats the original DQN model. No losses in either seat. |
| DQN_4_2 | 50 / 50 · **100%** | Same outcome against the retrained DQN checkpoint. |
| DQN_5_4 | 50 / 50 · **100%** | Third DQN variant, same result. QuorZero is 150 / 150 across all three DQN engines combined. |

---

## 6. QuorZero: Installation & Usage

### Prerequisites

| Component | Version |
|---|---|
| OS | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Python | 3.10 |
| TensorFlow | `tensorflow[and-cuda]` (GPU optional) |

### Install system dependencies

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep python3-pip \
  ros-humble-moveit-ros-planning-interface \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  build-essential cmake

pip3 install --upgrade pip
pip3 install pyrealsense2 opencv-python
pip3 install --timeout 600 --retries 5 "tensorflow[and-cuda]"
pip3 install tensorboard
```

### Clone and build

```bash
mkdir -p ~/rs2_ws/src
cd ~/rs2_ws/src
git clone https://github.com/BinethAtukorala/RS2-Quoridor.git

cd ~/rs2_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --packages-select quoridor_alphazero --symlink-install
source install/setup.bash
```

### Play against QuorZero

```bash
ros2 launch quoridor_alphazero play_vs_az.launch.py \
    model_dir:=$HOME/rs2_ws/quoridor_az_models/run1 \
    simulations:=400
```

Open the web UI at **http://localhost:8088**.

### Train from scratch

```bash
python3 -m quoridor_alphazero.train \
    --episodes 5000 --simulations 64 --updates-per-game 8 \
    --model-dir $HOME/rs2_ws/quoridor_az_models/run1 \
    --tb-log-dir $HOME/rs2_ws/quoridor_az_tensorboard/run1
```

### Resume training from a checkpoint

```bash
python3 -m quoridor_alphazero.train --resume \
    --model-dir $HOME/rs2_ws/quoridor_az_models/run1 \
    --tb-log-dir $HOME/rs2_ws/quoridor_az_tensorboard/run1_2 \
    --batch-size 1024 --lr-resume 8e-4
```

### View training metrics

```bash
tensorboard --logdir=$HOME/rs2_ws/quoridor_az_tensorboard
```

### Configurable parameters (`ai_move_node`)

| Parameter | Default | Effect |
|---|---|---|
| `model_dir` | — | Folder containing `az_net.weights.h5` |
| `simulations` | 200–400 | MCTS simulations per move — more = stronger but slower |
| `c_puct` | 1.5 | PUCT exploration weight |
| `filters / blocks` | 32 / 3 | Network shape — must match the trained checkpoint |

> **Note:** Only one move engine should run at a time. Running both `move_decision` (minimax) and `ai_move_node` simultaneously causes them to race on `/quoridor/compute_move_response`.

---

## 7. Robotic System

The physical layer that takes QuorZero's chosen move from a ROS 2 topic to a piece placed on the board. It consists of two subsystems: **Perception** (eyes) and **Robot Motion Control** (arms).

---

### 7.1. Perception

The perception subsystem uses RGB and depth video from an **Intel RealSense D435i** mounted eye-in-hand on the UR3e end-effector. It produces a live digital board state — pawn occupancy on a 5×5 grid and wall placement on a 4×4 grid — along with 3D world coordinates for every detected piece.

#### Nodes

| Node | Role |
|---|---|
| `camera_node` | Captures RGB and depth streams at 640×480 @ 15 FPS. Supports live input and ROS bag playback. Saves `camera_intrinsics.json` on first run. Exposes `/get_pawns` and `/get_walls` ROS 2 services for calibration fallback. |
| `perception_node` | Core detection. Applies a homography transform to produce a 500×500 top-down view, detects pawns via binary thresholding and walls via HSV red-segmentation. Publishes board state, wall state, 3D coordinates, and AR visualisation at 10 Hz. |
| `coordinate_node` | One-shot calibration. Detects all 25 pawn grid positions and 16 wall circle positions, bilinearly interpolates 3D world coordinates for every cell from 8 hand-measured corner poses, writes `pawn_coords.txt` and `wall_coords.txt`, then shuts down. |

#### Detection pipeline

```python
# Board localisation
gray  = clahe.apply(cv2.cvtColor(img, BGR2GRAY))
edges = cv2.Canny(GaussianBlur(gray), 50, 150)
# → find largest 4-point contour
# → getPerspectiveTransform → 500×500 top-down

# Pawn detection
_, mask = cv2.threshold(gray, 70, 255, BINARY_INV)
for each of 25 cells:
    if countNonZero(roi) / 1600 > 0.05:
        grid_state[r, c] = 1   # occupied

# Wall detection
red = inRange(hsv, [0,120,70],[10,255,255])
     | inRange(hsv, [170,120,70],[180,255,255])
for contour with area > 2000 px²:
    orient = "horiz" if w > h else "vert"
    wall_circles[r, c] = 2 if horiz else 1
```

#### Published topics

| Topic | Type | Content |
|---|---|---|
| `/perception/board_state` | `Int32MultiArray` (5×5) | Pawn occupancy — `1` = occupied, `0` = empty |
| `/perception/wall_state` | `Int32MultiArray` (4×4) | Wall placement — `0` empty, `1` vertical, `2` horizontal |
| `/perception/pawns_3d` | `Float32MultiArray` | 3D world coordinates `(row, col, x, y, z)` for detected pawns |
| `/perception/walls_inside_3d` | `Float32MultiArray` | 3D world coordinates for on-board walls |
| `/perception/augmented_reality` | `sensor_msgs/Image` | Live camera feed with AR overlays at 10 Hz |
| `/perception/topdown` | `sensor_msgs/Image` | Warped 500×500 top-down view at 10 Hz |

#### Running perception

```bash
# Step 1 — Calibration (one-shot; does not need camera_node running)
ros2 run perception coordinate_node

# Step 2 — Camera (live)
ros2 run perception camera_node

# Step 2 (alternative) — Camera (rosbag playback)
ros2 run perception camera_node --ros-args -p bag_file:=<path_to_bag>

# Step 3 — Perception
ros2 run perception perception_node
```

Verify detection by checking the AR overlay windows (blue boxes = pawns, red boxes = walls, green outline = board) and confirming topics:

```bash
ros2 topic echo /perception/board_state
ros2 topic echo /perception/wall_state
```

---

### 7.2. Robot Motion Control

The motion planning and control subsystem is the physical execution layer. It receives a piece type (`p`, `h`, or `v`) and start/end poses from game logic, then carries out the full pick-and-place pipeline using the **UR3e arm**, **MoveIt** for trajectory planning, and an **OnRobot RG2 gripper**.

#### Nodes

| Node | File | Role |
|---|---|---|
| `control` | `control.cpp` | Main action server on `/quoridor/bot_execute`. Executes the 11-step pick-and-place sequence. Retries failed grasps up to 3 times before aborting. |
| `gripper` | `gripper.cpp` | Standalone gripper node. Accepts `open`, `pickup_pawn`, `pickup_wall`, `drop_pawn`, `drop_wall` on `/gripper/command`. Performs stall detection during pickup and validates the held object width (pawn ~24 mm, wall ~17 mm) before publishing `success`, `fail`, or `wrong` to `/gripper/status`. |

#### Pick-and-place execution sequence

```
Step 1  → Perception waypoint (first move) + verify board_state received

Step 2P → Cartesian to start hover (pawn)
Step 2W → Joint move to wall rack slot (wall)

Step 3  → Cartesian descent: hover → contact
Step 4  → publishGripperCommand("pickup_*")
          retry up to MAX_PICKUP_RETRIES = 3

Step 5  → Cartesian ascent: contact → hover
Step 6  → (transit waypoint — optional)
Step 7  → Cartesian to end hover
Step 8  → Cartesian descent: hover → contact
Step 9  → publishGripperCommand("drop_*")
Step 10 → Cartesian ascent: contact → hover
Step 11 → Cartesian transit + joint snap
          → perception waypoint
          → verify board detection
```

#### Gripper stall detection

The gripper closes in 1 mm increments at 20 Hz. Once finger width drops below 15 mm, stall detection activates: if the measured width stops changing for 5 consecutive ticks (~250 ms), the gripper has made contact and stops closing immediately — preventing over-gripping. The final width is compared against expected values; a `drop` command is blocked if the held-object width differs from expected by more than 4× the tolerance.

#### Safety & workspace constraints

- **Ground plane** — a 3×3 m collision box added to the MoveIt planning scene at table height prevents any trajectory that would drive the end-effector into the table.
- **Joint constraints** — three shoulder and elbow joints are bounded via `JOINT_BOUNDS` to keep the arm above the board during all board-zone motions. Constraints are cleared only when reaching the wall rack, then immediately re-applied.
- **Cartesian minimum fraction** — contact-zone Cartesian paths require ≥95% coverage. If a path falls short, the node falls back through `MOVEMENT_WAYPOINT` and retries.

#### Running motion control

```bash
# Step 1 — Build
cd ~/rs2_ws
colcon build --packages-select control
source install/setup.bash

# Step 2 — Launch robot driver
ros2 launch ur_onrobot_control start_robot.launch.py \
    ur_type:=ur3e onrobot_type:=rg2 robot_ip:=192.168.0.194
# Wait until the terminal confirms the robot is ready.

# Step 3 — Launch MoveIt
ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py \
    ur_type:=ur3e onrobot_type:=rg2
# Wait until RViz opens and the robot model loads without errors.

# Step 4 — Launch control node
ros2 run control control

# Step 5 — Launch gripper node
ros2 run control gripper
```

**Optional — test a wall pickup slot without a full game:**

```bash
ros2 topic pub --once /quoridor/test_wall_pickup std_msgs/msg/Int32 "{data: 1}"
```

Drives the arm to wall rack slot 1–4 to verify joint configs before a match.

#### Configurable parameters (all in the tuning section at the top of `control.cpp`)

| Parameter | Default | Effect |
|---|---|---|
| `PERCEPTION_WAYPOINT` | joint angles (rad) | Where the arm rests between moves so the camera sees the board |
| `MOVEMENT_WAYPOINT` | joint angles (rad) | Transit waypoint for cross-board moves |
| `HOVER_OFFSET_M` | `0.08` m | Approach height above the contact point |
| `MAX_PICKUP_RETRIES` | `3` | Gripper retry attempts before aborting the action |
| `setMaxVelocityScalingFactor` | `0.3` | Arm speed (0.0–1.0) |
| `setMaxAccelerationScalingFactor` | `0.3` | Arm acceleration (0.0–1.0) |
| `setPlanningTime` | `10.0` s | Maximum MoveIt planning time per segment |
| `setNumPlanningAttempts` | `5` | MoveIt planning retries per segment |
| `JOINT_BOUNDS` | per-joint centre ± tolerance | Workspace constraint keeping motion above the board |

---

### 7.3. Full System Launch

Run each command in a separate terminal. Source the workspace in each one first:

```bash
source ~/rs2_ws/install/setup.bash
```

| Terminal | Command |
|---|---|
| 1 — Calibration (once) | `ros2 run perception coordinate_node` |
| 2 — Camera | `ros2 run perception camera_node` |
| 3 — Perception | `ros2 run perception perception_node` |
| 4 — Game logic + AI | `ros2 launch quoridor_alphazero play_vs_az.launch.py model_dir:=$HOME/rs2_ws/quoridor_az_models/run1 simulations:=400` |
| 5 — Robot driver | `ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 robot_ip:=192.168.0.194` |
| 6 — MoveIt | `ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py ur_type:=ur3e onrobot_type:=rg2` |
| 7 — Control | `ros2 run control control` |
| 8 — Gripper | `ros2 run control gripper` |

Once all nodes are running, open **http://localhost:8088** and press **Start** to begin a game.

---

## 8. Hardware Requirements

| Item | Qty | Notes |
|---|---|---|
| UR3e robotic arm | 1 | With teach pendant; set to Remote Control mode |
| OnRobot RG2 gripper | 1 | Mounted on the UR3e end-effector |
| Intel RealSense D435i | 1 | Eye-in-hand — mounted on the UR3e wrist |
| Custom 3D-printed camera mount | 1 | One-piece collar + camera plate |
| Fasteners: 11× M3 nuts and bolts | 1 set | Collar clamp, camera mount, board mounting |
| 5×5 Quoridor board | 1 | Bolted to the workspace table |
| Quoridor pawns | 2 | One per side |
| Quoridor walls | 8 | 4 per side in designated rack |
| USB 3.0 cable | 1 | Camera to workstation (USB 2.0 also works at reduced AR resolution) |
| Workstation PC | 1 | Ubuntu 22.04, min 8 GB RAM; NVIDIA GPU optional for training |

---

## 9. Troubleshooting

### QuorZero / Game Logic

**Bot stuck "thinking" indefinitely**
Only one move engine should be running. Check `ros2 node list` and confirm either `move_decision` or `ai_move_node` is present — not both. Check that node's log for a missing `az_net.weights.h5` checkpoint file.

**"No 3D coord for pawn end (x,y) — cannot send bot_execute goal"**
Perception hasn't reported 3D coordinates for that cell yet and the calibration fallback service isn't available. Start perception nodes before the state manager, and ensure `/get_pawns` and `/get_walls` services are live first.

**Web UI does not load at localhost:8088**
Confirm `web_interface` is running (`ros2 node list | grep web_interface`). Check whether the port is in use (`ss -ltn | grep 8088`) and override if needed with `--ros-args -p port:=<new_port>`.

**"Not the player's turn — move ignored"**
Moves out of turn are refused by `state_manager`. Check `current_turn` in `/quoridor/board_state`. Press **Bot first** in the UI if the bot should move first.

**Perception keeps re-triggering the same move**
Toggle input mode to manual and back to perception in the web UI to reset the `last_applied` snapshot.

### Perception

**`pawn_coords.txt` / `wall_coords.txt` / `camera_intrinsics.json` not found**
Run `coordinate_node` first (coordinate files), then `camera_node` (intrinsics file), before starting `perception_node`.

**`coordinate_node` stalls without printing "SUCCESS"**
Improve lighting or slightly reposition the board until all 25 grids and 16 circles are visible. If the "Waiting..." message appears more than ~10 times with no success, the board is not fully detected.

**`RuntimeError: Device already in use`**
Another process (`realsense-viewer` or another node) is holding the camera. Close it and retry.

### Motion Planning & Control

**Arm does not move to the perception waypoint at startup**
MoveIt is not running. Check `ros2 node list | grep move_group`. If absent, launch MoveIt before the control node.

**Node hangs after a gripper command**
The gripper node is not running. Check `ros2 node list | grep gripper`; if absent, run `ros2 run control gripper`.

**MoveIt reports "No solution found" for a board square**
The workspace joint constraints may be too tight. Temporarily comment out `setPathConstraints()` in `control.cpp` and retest. If planning succeeds, widen the tolerance on the relevant joint in `JOINT_BOUNDS`.

**Only 4 wall moves work per game**
The wall slot counter resets only on node restart. Restart `ros2 run control control` between games.

**How do I update board coordinates after repositioning the board?**
No changes to `control.cpp` are needed. Re-run `coordinate_node` to regenerate the calibration files — board coordinates arrive in the action goal from the perception subsystem.
