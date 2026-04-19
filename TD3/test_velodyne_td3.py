import argparse
import csv
import math
import os
import time

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from velodyne_env import GazeboEnv


class Actor(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Actor, self).__init__()

        self.layer_1 = nn.Linear(state_dim, 800)
        self.layer_2 = nn.Linear(800, 600)
        self.layer_3 = nn.Linear(600, action_dim)
        self.tanh = nn.Tanh()

    def forward(self, s):
        s = F.relu(self.layer_1(s))
        s = F.relu(self.layer_2(s))
        a = self.tanh(self.layer_3(s))
        return a


class TD3(object):
    def __init__(self, state_dim, action_dim):
        self.actor = Actor(state_dim, action_dim).to(device)

    def get_action(self, state):
        state = torch.Tensor(state.reshape(1, -1)).to(device)
        return self.actor(state).cpu().data.numpy().flatten()

    def load(self, filename, directory):
        self.actor.load_state_dict(
            torch.load("%s/%s_actor.pth" % (directory, filename), map_location=device)
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Evaluate a trained TD3 policy and export metrics to CSV."
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=20,
        help="Number of evaluation episodes to run.",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=500,
        help="Maximum number of steps per episode.",
    )
    parser.add_argument(
        "--model-name",
        default="TD3_velodyne",
        help="Base filename of the saved actor model.",
    )
    parser.add_argument(
        "--model-dir",
        default="./pytorch_models",
        help="Directory containing the saved actor model.",
    )
    parser.add_argument(
        "--launch-file",
        default="multi_robot_scenario.launch",
        help="Launch file passed to GazeboEnv.",
    )
    parser.add_argument(
        "--csv-path",
        default="./results/td3_evaluation_metrics.csv",
        help="CSV path for per-episode metrics and final summary.",
    )
    parser.add_argument(
        "--warmup-seconds",
        type=float,
        default=5.0,
        help="Seconds to wait after Gazebo launch before evaluation starts.",
    )
    parser.add_argument(
        "--manual-goal",
        action="store_true",
        help="Wait for a goal selected in RViz instead of sampling a random goal.",
    )
    parser.add_argument(
        "--move-boxes",
        action="store_true",
        help="Randomly reposition the cardboard boxes on each reset.",
    )
    parser.add_argument(
        "--dynamic-boxes",
        action="store_true",
        help="Continuously move the cardboard boxes to simulate dynamic obstacles.",
    )

    return parser.parse_args()


def euclidean_distance(point_a, point_b):
    return math.hypot(point_b[0] - point_a[0], point_b[1] - point_a[1])


def build_episode_result(
    episode_index,
    outcome,
    target,
    collision,
    episode_reward,
    episode_steps,
    path_length,
    start_pos,
    goal_pos,
    replanning_times,
):
    straight_line_distance = euclidean_distance(start_pos, goal_pos)
    if path_length <= 0:
        path_optimality = 0.0
    else:
        path_optimality = min(straight_line_distance / path_length, 1.0)

    avg_replanning_time = (
        float(sum(replanning_times) / len(replanning_times)) if replanning_times else 0.0
    )
    replanning_rate_hz = 1.0 / avg_replanning_time if avg_replanning_time > 0 else 0.0

    return {
        "row_type": "episode",
        "episode": episode_index,
        "outcome": outcome,
        "goal_reached": int(target),
        "collision": int(collision),
        "episode_reward": float(episode_reward),
        "episode_steps": int(episode_steps),
        "start_x": float(start_pos[0]),
        "start_y": float(start_pos[1]),
        "goal_x": float(goal_pos[0]),
        "goal_y": float(goal_pos[1]),
        "straight_line_distance_m": float(straight_line_distance),
        "path_length_m": float(path_length),
        "path_optimality": float(path_optimality),
        "avg_replanning_time_s": float(avg_replanning_time),
        "replanning_rate_hz": float(replanning_rate_hz),
    }


def build_summary(results):
    episode_count = len(results)
    goals_reached = sum(row["goal_reached"] for row in results)
    collisions = sum(row["collision"] for row in results)
    successful_path_rows = [row for row in results if row["goal_reached"]]

    mean_path_optimality = (
        float(
            sum(row["path_optimality"] for row in successful_path_rows)
            / len(successful_path_rows)
        )
        if successful_path_rows
        else 0.0
    )
    mean_replanning_time = float(
        sum(row["avg_replanning_time_s"] for row in results) / episode_count
    )
    mean_replanning_rate = float(
        sum(row["replanning_rate_hz"] for row in results) / episode_count
    )

    return {
        "row_type": "summary",
        "episode": "all",
        "outcome": "aggregate",
        "goal_reached": goals_reached,
        "collision": collisions,
        "episode_reward": float(sum(row["episode_reward"] for row in results) / episode_count),
        "episode_steps": int(sum(row["episode_steps"] for row in results)),
        "start_x": "",
        "start_y": "",
        "goal_x": "",
        "goal_y": "",
        "straight_line_distance_m": "",
        "path_length_m": "",
        "path_optimality": mean_path_optimality,
        "avg_replanning_time_s": mean_replanning_time,
        "replanning_rate_hz": mean_replanning_rate,
        "episodes": episode_count,
        "goal_reach_rate": float(goals_reached / episode_count),
        "collision_count": collisions,
    }


def write_csv(csv_path, results, summary):
    fieldnames = [
        "row_type",
        "episode",
        "outcome",
        "goal_reached",
        "collision",
        "episode_reward",
        "episode_steps",
        "start_x",
        "start_y",
        "goal_x",
        "goal_y",
        "straight_line_distance_m",
        "path_length_m",
        "path_optimality",
        "avg_replanning_time_s",
        "replanning_rate_hz",
        "episodes",
        "goal_reach_rate",
        "collision_count",
    ]

    csv_dir = os.path.dirname(csv_path)
    if csv_dir:
        os.makedirs(csv_dir, exist_ok=True)
    with open(csv_path, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for row in results:
            writer.writerow(row)
        writer.writerow(summary)


device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
seed = 0


def main():
    args = parse_args()

    torch.manual_seed(seed)
    np.random.seed(seed)

    environment_dim = 20
    robot_dim = 4
    state_dim = environment_dim + robot_dim
    action_dim = 2
    env = GazeboEnv(
        args.launch_file,
        environment_dim,
        manual_goal=args.manual_goal,
        move_boxes=args.move_boxes,
        dynamic_boxes=args.dynamic_boxes,
    )
    
    time.sleep(args.warmup_seconds)

    network = TD3(state_dim, action_dim)
    try:
        network.load(args.model_name, args.model_dir)
    except Exception as exc:
        raise ValueError("Could not load the stored model parameters") from exc

    results = []

    for episode_index in range(1, args.episodes + 1):
        state = env.reset()
        start_pos = (float(env.odom_x), float(env.odom_y))
        goal_pos = (float(env.goal_x), float(env.goal_y))
        prev_pos = start_pos

        done = False
        episode_steps = 0
        episode_reward = 0.0
        path_length = 0.0
        replanning_times = []
        collision = False
        target = False

        while not done and episode_steps < args.max_steps:
            action_start = time.perf_counter()
            action = network.get_action(np.array(state))
            replanning_times.append(time.perf_counter() - action_start)

            a_in = [(action[0] + 1) / 2, action[1]]
            next_state, reward, env_done, target = env.step(a_in)

            current_pos = (float(env.odom_x), float(env.odom_y))
            path_length += euclidean_distance(prev_pos, current_pos)
            prev_pos = current_pos

            episode_reward += reward
            episode_steps += 1

            collision = bool(reward <= -100.0 and not target)
            done = bool(env_done) or episode_steps >= args.max_steps
            state = next_state

        if target:
            outcome = "goal_reached"
        elif collision:
            outcome = "collision"
        else:
            outcome = "timeout"

        episode_result = build_episode_result(
            episode_index=episode_index,
            outcome=outcome,
            target=target,
            collision=collision,
            episode_reward=episode_reward,
            episode_steps=episode_steps,
            path_length=path_length,
            start_pos=start_pos,
            goal_pos=goal_pos,
            replanning_times=replanning_times,
        )
        results.append(episode_result)

        print(
            "Episode %d/%d | outcome=%s | goal=%d | collision=%d | "
            "path_optimality=%.4f | replanning_time=%.6fs"
            % (
                episode_index,
                args.episodes,
                outcome,
                episode_result["goal_reached"],
                episode_result["collision"],
                episode_result["path_optimality"],
                episode_result["avg_replanning_time_s"],
            )
        )

    summary = build_summary(results)
    write_csv(args.csv_path, results, summary)

    print("\nEvaluation summary")
    print("CSV saved to: %s" % args.csv_path)
    print("Goal reach rate: %.4f" % summary["goal_reach_rate"])
    print("Collision count: %d" % summary["collision_count"])
    print("Path optimality: %.4f" % summary["path_optimality"])
    print("Re-planning speed: %.6fs (%.2f Hz)" % (
        summary["avg_replanning_time_s"],
        summary["replanning_rate_hz"],
    ))


if __name__ == "__main__":
    main()
