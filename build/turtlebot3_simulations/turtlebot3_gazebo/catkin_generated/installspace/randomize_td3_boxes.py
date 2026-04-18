#!/usr/bin/env python3

import math
import random

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from rosgraph_msgs.msg import Clock


class TD3BoxRandomizer:
    def __init__(self):
        self.box_count = rospy.get_param("~box_count", 4)
        self.robot_model_name = rospy.get_param("~robot_model_name", "turtlebot3")
        self.box_prefix = rospy.get_param("~box_prefix", "cardboard_box_")
        self.min_box_separation = rospy.get_param("~min_box_separation", 0.9)
        self.min_distance_to_robot = rospy.get_param("~min_distance_to_robot", 1.0)
        self.oscillation_amplitude = rospy.get_param("~oscillation_amplitude", 0.6)
        self.oscillation_speed = rospy.get_param("~oscillation_speed", 0.8)
        self.update_rate = rospy.get_param("~update_rate", 20.0)
        self.last_robot_pose = None
        self.last_clock = None
        self.randomized_once = False
        self.box_configs = []

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback, queue_size=1)
        rospy.Subscriber("/clock", Clock, self.clock_callback, queue_size=1)

        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        # Give Gazebo a brief moment to finish spawning models before the first move.
        rospy.Timer(rospy.Duration(1.5), self.initial_randomize_callback, oneshot=True)
        rospy.Timer(rospy.Duration(1.0 / self.update_rate), self.update_boxes)

    @staticmethod
    def check_pos(x, y):
        goal_ok = True

        if -3.8 > x > -6.2 and 6.2 > y > 3.8:
            goal_ok = False

        if -1.3 > x > -2.7 and 4.7 > y > -0.2:
            goal_ok = False

        if -0.3 > x > -4.2 and 2.7 > y > 1.3:
            goal_ok = False

        if -0.8 > x > -4.2 and -2.3 > y > -4.2:
            goal_ok = False

        if -1.3 > x > -3.7 and -0.8 > y > -2.7:
            goal_ok = False

        if 4.2 > x > 0.8 and -1.8 > y > -3.2:
            goal_ok = False

        if 4 > x > 2.5 and 0.7 > y > -3.2:
            goal_ok = False

        if 6.2 > x > 3.8 and -3.3 > y > -4.2:
            goal_ok = False

        if 4.2 > x > 1.3 and 3.7 > y > 1.5:
            goal_ok = False

        if -3.0 > x > -7.2 and 0.5 > y > -1.5:
            goal_ok = False

        if x > 4.5 or x < -4.5 or y > 4.5 or y < -4.5:
            goal_ok = False

        return goal_ok

    def model_states_callback(self, msg):
        if self.robot_model_name not in msg.name:
            return

        robot_index = msg.name.index(self.robot_model_name)
        self.last_robot_pose = msg.pose[robot_index]

    def clock_callback(self, msg):
        current_time = msg.clock
        if self.last_clock is not None and current_time < self.last_clock:
            rospy.loginfo("Detected Gazebo world reset, randomizing TD3 cardboard boxes again.")
            self.randomize_boxes()
        self.last_clock = current_time

    def initial_randomize_callback(self, _event):
        self.randomize_boxes()

    def is_far_enough(self, x, y, chosen_positions):
        for other_x, other_y, _other_yaw in chosen_positions:
            if math.hypot(x - other_x, y - other_y) < self.min_box_separation:
                return False

        if self.last_robot_pose is not None:
            robot_x = self.last_robot_pose.position.x
            robot_y = self.last_robot_pose.position.y
            if math.hypot(x - robot_x, y - robot_y) < self.min_distance_to_robot:
                return False

        return True

    def corridor_is_valid(self, center_x, center_y):
        samples = 9
        for index in range(samples):
            if samples == 1:
                offset = 0.0
            else:
                ratio = float(index) / float(samples - 1)
                offset = -self.oscillation_amplitude + 2.0 * self.oscillation_amplitude * ratio

            candidate_x = center_x + offset
            if not self.check_pos(candidate_x, center_y):
                return False

        return True

    def sample_positions(self):
        positions = []
        max_attempts = 500

        for _ in range(self.box_count):
            found = False
            for _attempt in range(max_attempts):
                x = random.uniform(-4.5, 4.5)
                y = random.uniform(-4.5, 4.5)
                if not self.check_pos(x, y):
                    continue
                if not self.corridor_is_valid(x, y):
                    continue
                if not self.is_far_enough(x, y, positions):
                    continue

                yaw = random.uniform(-math.pi, math.pi)
                positions.append((x, y, yaw))
                found = True
                break

            if not found:
                raise RuntimeError("Could not find a valid random pose for every cardboard box.")

        return positions

    def move_box(self, model_name, x, y, yaw):
        state = ModelState()
        state.model_name = model_name
        state.reference_frame = "world"
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = 0.15
        state.pose.orientation.z = math.sin(yaw / 2.0)
        state.pose.orientation.w = math.cos(yaw / 2.0)

        response = self.set_model_state(state)
        if not response.success:
            raise RuntimeError("Failed to move {}: {}".format(model_name, response.status_message))

    def randomize_boxes(self):
        try:
            positions = self.sample_positions()
            self.box_configs = []
            for index, (x, y, yaw) in enumerate(positions):
                box_phase = random.uniform(0.0, 2.0 * math.pi)
                model_name = "{}{}".format(self.box_prefix, index)
                self.box_configs.append(
                    {
                        "model_name": model_name,
                        "center_x": x,
                        "center_y": y,
                        "yaw": yaw,
                        "phase": box_phase,
                    }
                )
                self.move_box(model_name, x, y, yaw)
            self.randomized_once = True
            rospy.loginfo("Randomized %d TD3 cardboard boxes.", self.box_count)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "TD3 box randomization skipped: %s", exc)

    def update_boxes(self, _event):
        if not self.randomized_once:
            return

        current_time = rospy.get_time()
        if current_time <= 0.0:
            return

        try:
            for box_config in self.box_configs:
                offset = self.oscillation_amplitude * math.sin(
                    self.oscillation_speed * current_time + box_config["phase"]
                )
                x = box_config["center_x"] + offset
                y = box_config["center_y"]
                self.move_box(box_config["model_name"], x, y, box_config["yaw"])
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "TD3 box motion skipped: %s", exc)


if __name__ == "__main__":
    rospy.init_node("td3_box_randomizer")
    TD3BoxRandomizer()
    rospy.spin()
