#!/usr/bin/env python3

import math

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from rosgraph_msgs.msg import Clock


class BookstoreBoxMover:
    def __init__(self):
        self.oscillation_amplitude = rospy.get_param("~oscillation_amplitude", 0.5)
        self.oscillation_speed = rospy.get_param("~oscillation_speed", 0.8)
        self.update_rate = rospy.get_param("~update_rate", 20.0)
        self.last_clock = None

        self.box_configs = [
            {"model_name": "cardboard_box_0", "center_x": -1.8, "center_y": -5.8, "yaw": 0.0, "phase": 0.0},
            {"model_name": "cardboard_box_1", "center_x": -5.8, "center_y": -2.2, "yaw": 0.0, "phase": math.pi / 2.0},
            {"model_name": "cardboard_box_2", "center_x": -2.5, "center_y": 5.8, "yaw": 0.0, "phase": math.pi},
            {"model_name": "cardboard_box_3", "center_x": 3.0, "center_y": 1.5, "yaw": 0.0, "phase": 3.0 * math.pi / 2.0},
        ]

        rospy.Subscriber("/clock", Clock, self.clock_callback, queue_size=1)

        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        rospy.Timer(rospy.Duration(1.0), self.initialize_boxes, oneshot=True)
        rospy.Timer(rospy.Duration(1.0 / self.update_rate), self.update_boxes)

    def clock_callback(self, msg):
        current_time = msg.clock
        if self.last_clock is not None and current_time < self.last_clock:
            rospy.loginfo("Detected Gazebo world reset, restoring bookstore box motion.")
            self.reset_boxes()
        self.last_clock = current_time

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

    def reset_boxes(self):
        for box_config in self.box_configs:
            self.move_box(
                box_config["model_name"],
                box_config["center_x"],
                box_config["center_y"],
                box_config["yaw"],
            )

    def initialize_boxes(self, _event):
        try:
            self.reset_boxes()
            rospy.loginfo("Initialized bookstore cardboard boxes for oscillating motion.")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Bookstore box initialization skipped: %s", exc)

    def update_boxes(self, _event):
        current_time = rospy.get_time()
        if current_time <= 0.0:
            return

        try:
            for box_config in self.box_configs:
                offset = self.oscillation_amplitude * math.sin(
                    self.oscillation_speed * current_time + box_config["phase"]
                )
                self.move_box(
                    box_config["model_name"],
                    box_config["center_x"] + offset,
                    box_config["center_y"],
                    box_config["yaw"],
                )
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Bookstore box motion skipped: %s", exc)


if __name__ == "__main__":
    rospy.init_node("bookstore_box_mover")
    BookstoreBoxMover()
    rospy.spin()
