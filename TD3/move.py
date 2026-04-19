#!/usr/bin/env python3

import math

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from rosgraph_msgs.msg import Clock


class BookstoreBoxMover:
    def __init__(self):
        self.oscillation_amplitude = rospy.get_param("~oscillation_amplitude", 1)
        self.oscillation_speed = rospy.get_param("~oscillation_speed", 0.25)
        self.update_rate = rospy.get_param("~update_rate", 20.0)
        self.last_clock = None

        self.box_configs = [
            {
                "model_name": "cardboard_box_0",
                "center_x": -1.8,
                "center_y": -5.8,
                "yaw": 0.0,
                "phase": 0.0,
                "x_amplitude": 3.2,
                "y_amplitude": 0.0,
                "speed_scale": 0.9,
                "path_type": "horizontal",
            },
            {
                "model_name": "cardboard_box_1",
                "center_x": -5.8,
                "center_y": -2.2,
                "yaw": 0.0,
                "phase": math.pi / 2.0,
                "x_amplitude": 0.0,
                "y_amplitude": 2.8,
                "speed_scale": 0.65,
                "path_type": "vertical",
            },
            {
                "model_name": "cardboard_box_2",
                "center_x": -2.5,
                "center_y": 5.8,
                "yaw": 0.0,
                "phase": math.pi,
                "x_amplitude": 2.8,
                "y_amplitude": 1.9,
                "speed_scale": 0.55,
                "path_type": "diagonal",
            },
            {
                "model_name": "cardboard_box_3",
                "center_x": 3.0,
                "center_y": 1.5,
                "yaw": 0.0,
                "phase": 3.0 * math.pi / 2.0,
                "x_amplitude": 2.6,
                "y_amplitude": 1.7,
                "speed_scale": 0.7,
                "path_type": "ellipse",
            },
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

    def compute_position(self, box_config, current_time):
        phase = self.oscillation_speed * box_config["speed_scale"] * current_time + box_config["phase"]
        amplitude_scale = self.oscillation_amplitude / 1.4
        x_amplitude = box_config["x_amplitude"] * amplitude_scale
        y_amplitude = box_config["y_amplitude"] * amplitude_scale
        path_type = box_config["path_type"]

        if path_type == "horizontal":
            x = box_config["center_x"] + x_amplitude * math.sin(phase)
            y = box_config["center_y"]
        elif path_type == "vertical":
            x = box_config["center_x"]
            y = box_config["center_y"] + y_amplitude * math.sin(phase)
        elif path_type == "diagonal":
            x = box_config["center_x"] + x_amplitude * math.sin(phase)
            y = box_config["center_y"] + y_amplitude * math.sin(phase)
        else:
            x = box_config["center_x"] + x_amplitude * math.sin(phase)
            y = box_config["center_y"] + y_amplitude * math.cos(phase)

        return x, y

    def update_boxes(self, _event):
        current_time = rospy.get_time()
        if current_time <= 0.0:
            return

        try:
            for box_config in self.box_configs:
                x, y = self.compute_position(box_config, current_time)
                self.move_box(
                    box_config["model_name"],
                    x,
                    y,
                    box_config["yaw"],
                )
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Bookstore box motion skipped: %s", exc)


if __name__ == "__main__":
    rospy.init_node("bookstore_box_mover")
    BookstoreBoxMover()
    rospy.spin()