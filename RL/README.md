# TD3-based Obstacle Avoidance (Gazebo + Velodyne)

A reinforcement learning pipeline for training and evaluating a **Twin Delayed DDPG (TD3)** agent for continuous-control obstacle avoidance in Gazebo, using a Velodyne-like pointcloud sensor.

---

## Table of Contents

- [Project Structure](#project-structure)
- [Quick Start](#quick-start)
- [Training](#training)
- [Evaluation](#evaluation)
- [Environment Details](#environment-details)
- [State, Action & Reward](#state-action--reward)
- [TD3 Architecture & Hyperparameters](#td3-architecture--hyperparameters)
- [Outputs](#outputs)
- [Tips & Troubleshooting](#tips--troubleshooting)
- [Contributing](#contributing)
- [References](#references)

---

## Project Structure

```
RL/
├── TD3/
│   ├── train_velodyne_td3.py     # Training loop and TD3 implementation
│   ├── test_velodyne_td3.py      # Evaluation script; exports per-episode metrics to CSV
│   ├── velodyne_env.py           # Gazebo environment wrapper (gym-like: reset / step)
│   ├── replay_buffer.py          # Replay buffer used during training
│   ├── move.py                   # Helper/utility functions for motion
│   ├── assets/                   # Launch files and other assets for velodyne_env.py
│   ├── pytorch_models/           # (created at runtime) Saved actor/critic weights
│   └── results/                  # (created at runtime) Evaluation results and CSVs
├── multi_robot_scenario/         # ROS package — robot URDFs, launch files, meshes
└── velodyne_simulator/           # Velodyne plugin, URDFs, docs, and GPU notes
```

---

## Quick Start

### 1. Prerequisites

Ensure **ROS** and **Gazebo** are installed and configured for your distro (Melodic or Noetic).

### 2. Build the ROS Package

```bash
cd ~/catkin_ws
# Copy or symlink RL/multi_robot_scenario into ~/catkin_ws/src/
catkin_make
source devel/setup.bash
```

### 3. Install Python Dependencies

```bash
pip3 install numpy torch squaternion tensorboard
```

> **Note:** ROS Python packages (`rospy`, `sensor_msgs`, `gazebo_msgs`, `nav_msgs`, `visualization_msgs`) come from your ROS installation and should **not** be pip-installed.

### 4. Launch File Location

- Place the launch file under `RL/TD3/assets/` (default lookup path), **or**
- Pass an absolute path via `--launch-file` (recommended for catkin workspaces).

---

## Training

Run from the repository root:

```bash
python3 RL/TD3/train_velodyne_td3.py
```

**Behavior:**

- Spawns `roscore` and `roslaunch` internally on port `11311` — do **not** run a conflicting `roscore` on the same port.
- Saves actor and critic weights to `RL/TD3/pytorch_models/` as `<file_name>_actor.pth` and `<file_name>_critic.pth` (default: `TD3_velodyne`).
- Saves evaluation snapshots (numpy arrays) to `RL/TD3/results/`.
- Logs metrics to TensorBoard via `SummaryWriter`.

---

## Evaluation

```bash
python3 RL/TD3/test_velodyne_td3.py \
  --episodes 20 \
  --max-steps 500 \
  --model-name TD3_velodyne \
  --model-dir ./RL/TD3/pytorch_models \
  --launch-file /full/path/to/your/multi_robot_scenario.launch \
  --csv-path ./RL/TD3/results/td3_evaluation_metrics.csv \
  --warmup-seconds 5.0 \
  --move-boxes \
  --dynamic-boxes
```

### Flags

| Flag | Type | Default | Description |
|---|---|---|---|
| `--episodes` | int | `20` | Number of episodes to run |
| `--max-steps` | int | `500` | Maximum steps per episode |
| `--model-name` | str | `TD3_velodyne` | Base filename of saved actor model |
| `--model-dir` | str | `./pytorch_models` | Directory containing model files |
| `--launch-file` | str | `multi_robot_scenario.launch` | Launch file passed to `GazeboEnv` |
| `--csv-path` | str | `./results/td3_evaluation_metrics.csv` | Path to write per-episode CSV results |
| `--warmup-seconds` | float | — | Seconds to wait after launch before starting |
| `--manual-goal` | flag | — | Wait for a goal from RViz (`/move_base_simple/goal` or `/clicked_point`) |
| `--move-boxes` | flag | — | Reposition cardboard boxes on reset |
| `--dynamic-boxes` | flag | — | Move cardboard boxes during episodes |

The script writes per-episode rows **and** a summary row to the CSV at `--csv-path`.

---

## Environment Details

**File:** `velodyne_env.py`

### Subscriptions

| Topic | Message Type | Purpose |
|---|---|---|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | Converted to 1D laser-like vector of length `environment_dim` |
| `/r1/odom` | `nav_msgs/Odometry` | Robot pose & orientation |
| `/move_base_simple/goal`, `/clicked_point` | — | Manual goal input (if `manual_goal=True`) |

### Publications

| Topic | Message Type | Purpose |
|---|---|---|
| `/r1/cmd_vel` | `Twist` | Velocity commands |
| `goal_point`, `linear_velocity`, `angular_velocity` | `MarkerArray` | Visual markers |

### World

- Bounded workspace: roughly **−4.5 m to +4.5 m** in x and y.
- 4 cardboard boxes (`cardboard_box_0` to `cardboard_box_3`) placed randomly or moved dynamically based on flags.

---

## State, Action & Reward

### State

| Component | Size | Description |
|---|---|---|
| Laser vector | `environment_dim` (default: 20) | Min range per angular bin |
| Robot state | 4 | `[distance_to_goal, angle_to_goal, last_linear_action, last_angular_action]` |
| **Total** | **24** | `state_dim = environment_dim + 4` |

### Action

A 2D continuous action vector (actor outputs tanh → `[-1, 1]`):

| Index | Description |
|---|---|
| `action[0]` | Linear velocity — converted as `(action[0] + 1) / 2` before sending |
| `action[1]` | Angular velocity — range `[-1, 1]` |

### Reward

```
if distance < 0.3 m:        reward = +100.0   (goal reached)
elif min_laser < 0.35 m:    reward = -100.0   (collision)
else:
    r3 = (1 - min_laser) if min_laser < 1 else 0
    reward = action[0]/2 - |action[1]|/2 - r3/2
```

| Threshold | Value |
|---|---|
| `GOAL_REACHED_DIST` | 0.3 m |
| `COLLISION_DIST` | 0.35 m |

---

## TD3 Architecture & Hyperparameters

### Networks

**Actor:**
```
Linear(state_dim → 800) → ReLU → Linear(800 → 600) → ReLU → Linear(600 → action_dim) → Tanh
```

**Critic:** Two independent Q-networks, each taking state + action and outputting a scalar Q-value.

### Hyperparameters

| Parameter | Default |
|---|---|
| Device | `cuda` if available, else `cpu` |
| Seed | `0` |
| `eval_freq` | `5000` |
| `max_ep` | `500` |
| `eval_ep` | `10` |
| `max_timesteps` | `5,000,000` |
| `expl_noise` (start → min) | `1.0 → 0.1` |
| `expl_decay_steps` | `500,000` |
| `batch_size` | `40` |
| `discount` (γ) | `0.99999` |
| `tau` (soft update) | `0.005` |
| `policy_noise` | `0.2` |
| `noise_clip` | `0.5` |
| `policy_freq` | `2` |
| `buffer_size` | `1,000,000` |
| `file_name` | `TD3_velodyne` |
| `random_near_obstacle` | `True` |
| `move_boxes` | `True` |
| `dynamic_boxes` | `True` |

---

## Outputs

| Output | Location |
|---|---|
| Actor weights | `RL/TD3/pytorch_models/<file_name>_actor.pth` |
| Critic weights | `RL/TD3/pytorch_models/<file_name>_critic.pth` |
| Evaluation metrics (numpy) | `RL/TD3/results/<file_name>.npy` |
| Per-episode CSV | Path specified by `--csv-path` |
| TensorBoard logs | Default `SummaryWriter` directory |

---

## Tips & Troubleshooting

- **Slow Gazebo with large pointclouds:** Reduce `samples` or `hz` in the Velodyne URDF (see `RL/velodyne_simulator`).
- **Launch file not found:** Use an absolute path with `--launch-file`.
- **Port conflicts:** Do not run another `roscore` on port `11311` — the training/testing scripts spawn one internally.
- **RViz visualization:** Subscribe to `/velodyne_points`, `/r1/odom`, and the marker topics `goal_point`, `linear_velocity`, `angular_velocity`.

---

## Contributing

- Open issues or PRs for bugfixes, improved hyperparameters, or new environments.
- When adding launch files or URDFs, place them in `RL/TD3/assets/` or use an absolute path with `--launch-file`.

---

## References

- See `RL/velodyne_simulator/README.md` for details on Velodyne URDFs, plugin parameters, and known issues.
- [TD3 Paper — Addressing Function Approximation Error in Actor-Critic Methods](https://arxiv.org/abs/1802.09477)
